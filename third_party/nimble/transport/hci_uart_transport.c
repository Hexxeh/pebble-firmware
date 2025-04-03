/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// TODO: transport.h needs os_mbuf.h to be included first
// clang-format off
#include <os/os_mbuf.h>
// clang-format on

#include <board/board.h>
#include <drivers/uart.h>
#include <kernel/pebble_tasks.h>
#include <nimble/transport.h>
#include <nimble/transport/hci_h4.h>
#include <nimble/transport_impl.h>
#include <os/os_mempool.h>
#include <queue.h>
#include <system/passert.h>
#include <util/circular_buffer.h>
#include <util/math.h>

#define TX_Q_SIZE                                                                      \
  (MYNEWT_VAL(BLE_TRANSPORT_ACL_FROM_LL_COUNT) + MYNEWT_VAL(BLE_TRANSPORT_EVT_COUNT) + \
   MYNEWT_VAL(BLE_TRANSPORT_EVT_DISCARDABLE_COUNT))

extern void ble_chipset_init(void);
extern bool ble_chipset_start(void);

// Added to HCI task queue when data is received from UART
const uint8_t HCI_TASK_MSG_RX = 0x01;

// Added to HCI task queue when data is ready to be sent to UART or the UART sent a byte and is ready for another
const uint8_t HCI_TASK_MSG_TX = 0x02;

typedef uint8_t hci_task_msg;

static TaskHandle_t s_hci_task_handle;
static QueueHandle_t s_hci_task_queue;

#define DMA_BUFFER_LENGTH (200)
static uint8_t DMA_BSS s_dma_buffer[DMA_BUFFER_LENGTH] __attribute__((aligned(4)));

static CircularBuffer s_rx_buffer;
static uint8_t s_rx_storage[2048];
static SemaphoreHandle_t s_cmd_done;

static CircularBuffer s_tx_buffer;
static uint8_t s_tx_storage[1024];
static SemaphoreHandle_t s_tx_buffer_drained;

static struct hci_h4_sm hci_uart_h4sm;
static bool chipset_start_done = false;

static void prv_lock(void) { portENTER_CRITICAL(); }

static void prv_unlock(void) { portEXIT_CRITICAL(); }

static int hci_uart_frame_cb(uint8_t pkt_type, void *data) {
  xSemaphoreGive(s_cmd_done);

  // HACK: passing responses to commands Nimble didn't generate causes issues
  if (!chipset_start_done) {
    ble_transport_free(data);
    return 0;
  }

  switch (pkt_type) {
    case HCI_H4_ACL:
      return ble_transport_to_hs_acl(data);
    case HCI_H4_EVT:
      return ble_transport_to_hs_evt(data);
    case HCI_H4_ISO:
      return ble_transport_to_hs_iso(data);
    default:
      WTF;
  }

  return -1;
}

static int prv_get_next_tx_byte(void) {
  const uint8_t *data;
  uint16_t bytes_read;
  int rc;

  prv_lock();
  bool success = circular_buffer_read(&s_tx_buffer, 1, &data, &bytes_read);

  if (success) {
    rc = *data;
    circular_buffer_consume(&s_tx_buffer, 1);
  } else {
    rc = -1;
  }

  prv_unlock();

  return rc;
}

static bool prv_uart_tx_irq_handler(UARTDevice *dev) {
  BaseType_t should_context_switch = false;

  int byte = prv_get_next_tx_byte();
  if (byte >= 0) {
    uart_write_byte(BLUETOOTH_UART, byte);
  } else {
    uart_set_tx_interrupt_enabled(BLUETOOTH_UART, false);
  }

  xSemaphoreGiveFromISR(s_tx_buffer_drained, &should_context_switch);

  return should_context_switch;
}

static bool did_overrun = false;
static bool did_framing_error = false;

static bool prv_uart_rx_irq_handler(UARTDevice *dev, uint8_t data,
                                    const UARTRXErrorFlags *err_flags) {
  BaseType_t should_context_switch = false;

  if (err_flags->framing_error || err_flags->overrun_error) {
    PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_ERROR, "Bluetooth UART overrun:%d framing:%d",
              err_flags->overrun_error, err_flags->framing_error);
    if (err_flags->overrun_error) {
      did_overrun = true;
    }
    if (err_flags->framing_error) {
      did_framing_error = true;
    }
  }

  prv_lock();
  PBL_ASSERTN(circular_buffer_get_write_space_remaining(&s_rx_buffer) > 0);
  circular_buffer_write(&s_rx_buffer, &data, 1);
  xQueueSendFromISR(s_hci_task_queue, &HCI_TASK_MSG_RX, &should_context_switch);
  prv_unlock();

  return should_context_switch;
}

static uint8_t read_buf[64];
static void prv_hci_task_handle_rx(void) {
  int consumed_bytes;
  uint16_t bytes_remaining;
  while (true) {
    prv_lock();

    bytes_remaining = circular_buffer_get_read_space_remaining(&s_rx_buffer);
    if (bytes_remaining == 0) {
      prv_unlock();
      break;
    }

    bytes_remaining = MIN(sizeof(read_buf), bytes_remaining);
    circular_buffer_copy(&s_rx_buffer, &read_buf, bytes_remaining);
    prv_unlock();

    consumed_bytes = hci_h4_sm_rx(&hci_uart_h4sm, read_buf, bytes_remaining);
    PBL_ASSERTN(consumed_bytes >= 0);

    prv_lock();
    circular_buffer_consume(&s_rx_buffer, consumed_bytes);
    prv_unlock();
  }
}

static void prv_hci_task_handle_tx(void) {
  uart_set_tx_interrupt_enabled(BLUETOOTH_UART, true);
}

static void prv_hci_task_main(void *unused) {
  hci_task_msg msg;

  while (true) {
    xQueueReceive(s_hci_task_queue, &msg, portMAX_DELAY);

    switch (msg) {
      case HCI_TASK_MSG_RX:
        prv_hci_task_handle_rx();
        break;
      case HCI_TASK_MSG_TX:
        prv_hci_task_handle_tx();
        break;
    }
  }
}

void ble_transport_ll_init(void) {
  hci_h4_sm_init(&hci_uart_h4sm, &hci_h4_allocs_from_ll, hci_uart_frame_cb);

  s_hci_task_queue = xQueueCreate(8, sizeof(hci_task_msg));
  PBL_ASSERTN(s_hci_task_queue);

  s_tx_buffer_drained = xSemaphoreCreateBinary();
  s_cmd_done = xSemaphoreCreateBinary();

  circular_buffer_init(&s_rx_buffer, s_rx_storage, sizeof(s_rx_storage));
  circular_buffer_init(&s_tx_buffer, s_tx_storage, sizeof(s_tx_storage));
  s_rx_buffer.auto_reset = false;

  ble_chipset_init();

  uart_init(BLUETOOTH_UART);
  uart_set_baud_rate(BLUETOOTH_UART, 115200);
  uart_set_rx_interrupt_handler(BLUETOOTH_UART, prv_uart_rx_irq_handler);
  uart_set_tx_interrupt_handler(BLUETOOTH_UART, prv_uart_tx_irq_handler);
  uart_set_rx_interrupt_enabled(BLUETOOTH_UART, true);
  uart_start_rx_dma(BLUETOOTH_UART, s_dma_buffer, DMA_BUFFER_LENGTH);

  xSemaphoreGive(s_tx_buffer_drained);

  TaskParameters_t task_params = {
      .pvTaskCode = prv_hci_task_main,
      .pcName = "NimbleHCI",
      .usStackDepth = 4000 / sizeof(StackType_t), // TODO: can probably be reduced
      .uxPriority = (configMAX_PRIORITIES - 2) | portPRIVILEGE_BIT,
      .puxStackBuffer = NULL,
  };

  pebble_task_create(PebbleTask_BTHCI, &task_params, &s_hci_task_handle);
  PBL_ASSERTN(s_hci_task_handle);

  if (ble_chipset_start()) {
    chipset_start_done = true;
  }
}

static void prv_tx_flatbuf(uint8_t *buf, uint16_t len) {
  while (circular_buffer_get_write_space_remaining(&s_tx_buffer) < len) {
    xSemaphoreTake(s_tx_buffer_drained, portMAX_DELAY);
  }

  bool complete_write = circular_buffer_write(&s_tx_buffer, buf, len);
  PBL_ASSERTN(complete_write);

  xQueueSend(s_hci_task_queue, &HCI_TASK_MSG_TX, portMAX_DELAY);
}

static void prv_tx_mbuf(struct os_mbuf *om) {
  uint16_t len = OS_MBUF_PKTLEN(om);
  while (circular_buffer_get_write_space_remaining(&s_tx_buffer) < len) {
    xSemaphoreTake(s_tx_buffer_drained, portMAX_DELAY);
  }

  uint8_t *data;
  uint16_t available_space = circular_buffer_write_prepare(&s_tx_buffer, &data);
  PBL_ASSERTN(available_space >= len);
  os_mbuf_copydata(om, 0, len, data);
  circular_buffer_write_finish(&s_tx_buffer, len);

  xQueueSend(s_hci_task_queue, &HCI_TASK_MSG_TX, portMAX_DELAY);
}

void ble_queue_cmd(void *buf, bool wait) {
  uint8_t type = HCI_H4_CMD;
  uint16_t len = 3 + ((uint8_t *)buf)[2];

  prv_tx_flatbuf(&type, 1);
  prv_tx_flatbuf(buf, len);

  if (wait) {
    xSemaphoreTake(s_cmd_done, portMAX_DELAY);
  }
}

/* APIs to be implemented by HS/LL side of transports */
int ble_transport_to_ll_cmd_impl(void *buf) {
  ble_queue_cmd(buf, false);
  ble_transport_free(buf);

  return 0;
}

int ble_transport_to_ll_acl_impl(struct os_mbuf *om) {
  uint8_t type = HCI_H4_ACL;

  prv_tx_flatbuf(&type, 1);
  prv_tx_mbuf(om);
  os_mbuf_free_chain(om);

  return 0;
}

int ble_transport_to_ll_iso_impl(struct os_mbuf *om) {
  uint8_t type = HCI_H4_ISO;

  prv_tx_flatbuf(&type, 1);
  prv_tx_mbuf(om);
  os_mbuf_free_chain(om);

  return 0;
}
