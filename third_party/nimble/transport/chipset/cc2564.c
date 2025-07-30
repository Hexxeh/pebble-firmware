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

#include <board/board.h>
#include <drivers/exti.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <FreeRTOS.h>
#include <kernel/util/sleep.h>
#include <nimble/transport/hci_h4.h>
#include <resource/resource.h>
#include <resource/resource_ids.auto.h>
#include <resource/resource_mapped.h>
#include <semphr.h>
#include <system/logging.h>
#include <system/passert.h>

#define HCI_VS_SLEEP_MODE_CONFIG (0xFD0C)
#define HCI_VS_UPDATE_UART_HCI_BAUDRATE (0xFF36)
#define HCI_VS_HCILL_PARAMETERS (0XFD2B)
#define HCI_BAUD_RATE (921600)
#define HCILL_GO_TO_SLEEP_IND (0x30)
#define HCILL_GO_TO_SLEEP_ACK (0x31)
#define HCILL_WAKE_UP_IND (0x32)
#define HCILL_WAKE_UP_ACK (0x33)

#define BT_GPIO_EXTI_CONFIG \
  ((ExtiConfig) { BOARD_CONFIG_BT_COMMON.wakeup.int_exti.exti_port_source, \
                  BOARD_CONFIG_BT_COMMON.wakeup.int_exti.exti_line})

typedef enum {
  eHCILLStateAwake,
  eHCILLStateAsleep,
  eHCILLStateWaitForAck,
} eHCILLState;

typedef struct PACKED {
  uint8_t type;
  uint16_t opcode;
  uint8_t size;
  uint8_t data[];
} BTSHCICommand;

typedef struct PACKED { 
  uint8_t type;
  uint16_t opcode;
  uint8_t size;
  uint32_t baud_rate;
} BTSHCIUpdateBaudRateCommand;

typedef struct PACKED {
  uint8_t type;
  uint16_t opcode;
  uint8_t size;
  uint16_t inactivity_timeout; // in frames of 1.25ms
  uint16_t retransmit_timeout; // in frames of 1.25ms
  uint8_t rts_pulse_width; // in microseconds
} BTSHCIUpdateHCILLParametersCommand;

extern void ble_queue_cmd(void *buf, bool needs_free, bool wait);

static void ble_chipset_wakeup_interrupt_handler(bool *should_context_switch);
bool ble_chipset_ensure_awake(void);

static eHCILLState ehcill_state = eHCILLStateAwake;

static void prv_lock(void) { portENTER_CRITICAL(); }

static void prv_unlock(void) { portEXIT_CRITICAL(); }

static bool ble_run_bts(const ResAppNum bts_file) {
  size_t i = 0;
  size_t bts_len = 0;

  if (!resource_is_valid(SYSTEM_APP, bts_file)) {
    PBL_LOG_D(LOG_DOMAIN_BT_STACK, LOG_LEVEL_ERROR, "Can't load BT service pack: bad system resources!");
    return false;
  }

  PebbleTask task = pebble_task_get_current();
  resource_mapped_use(task);

  const uint8_t *bts_data =
      resource_get_readonly_bytes(SYSTEM_APP, bts_file, &bts_len, true /* is_privileged */);

  while (i < bts_len) {
    BTSHCICommand *command = (BTSHCICommand *)&bts_data[i];
    i += sizeof(BTSHCICommand) + command->size;

    // if (command->opcode == HCI_VS_SLEEP_MODE_CONFIG) {
    //   continue; // skip sleep mode config commands
    // }

    if (command->opcode == HCI_VS_UPDATE_UART_HCI_BAUDRATE) {
      PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "ble_bts: Setting baud rate to %d", HCI_BAUD_RATE);
      static BTSHCIUpdateBaudRateCommand baud_rate_command = {
          .opcode = HCI_VS_UPDATE_UART_HCI_BAUDRATE,
          .size = sizeof(uint32_t),
          .type = HCI_H4_CMD,
          .baud_rate = HCI_BAUD_RATE,
      };
      command = (BTSHCICommand *)&baud_rate_command;
    }

    ble_queue_cmd(&command->opcode, false, true);

    if (command->opcode == HCI_VS_UPDATE_UART_HCI_BAUDRATE) {
      uart_set_baud_rate(BLUETOOTH_UART, HCI_BAUD_RATE);
    }
  }

  resource_mapped_release(task);

  // static BTSHCIUpdateHCILLParametersCommand hcill_params_command = {
  //     .opcode = HCI_VS_HCILL_PARAMETERS,
  //     .size = 5,
  //     .type = HCI_H4_CMD,
  //     .inactivity_timeout = 80, // 100ms
  //     .retransmit_timeout = 80, // 100ms
  //     .rts_pulse_width = 150, // 150us
  // };
  // ble_queue_cmd(&hcill_params_command.opcode, false, true);

  return true;
}

void ble_chipset_init(UARTRXInterruptHandler rx_interrupt_handler, 
                      UARTTXInterruptHandler tx_interrupt_handler) {                        
  gpio_output_init(&BOARD_CONFIG_BT_COMMON.reset, GPIO_OType_PP, GPIO_Speed_25MHz);
  gpio_output_set(&BOARD_CONFIG_BT_COMMON.reset, true);
  psleep(100);
  gpio_output_set(&BOARD_CONFIG_BT_COMMON.reset, false);

  exti_configure_pin(BT_GPIO_EXTI_CONFIG, ExtiTrigger_Rising, ble_chipset_wakeup_interrupt_handler);

  uart_init(BLUETOOTH_UART);
  uart_set_baud_rate(BLUETOOTH_UART, 115200);
  uart_set_rx_interrupt_handler(BLUETOOTH_UART, rx_interrupt_handler);
  uart_set_tx_interrupt_handler(BLUETOOTH_UART, tx_interrupt_handler);
  uart_set_rx_interrupt_enabled(BLUETOOTH_UART, true);
  uart_set_tx_interrupt_enabled(BLUETOOTH_UART, true);
}

bool ble_chipset_start(void) {
  if (!ble_run_bts(RESOURCE_ID_BT_PATCH)) return false;

  // HACK: this is just here to let the service pack commands get processed before we continue
  psleep(500);

  PBL_LOG_D(LOG_DOMAIN_BT_STACK, LOG_LEVEL_INFO, "bts files sent");

  return true;
}

static void prv_return_to_awake(void) {
  ehcill_state = eHCILLStateAwake;
  uart_set_tx_interrupt_enabled(BLUETOOTH_UART, true);
}

static void ble_chipset_wakeup_interrupt_handler(bool *should_context_switch) {  
  PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "HCILL interrupt triggered!");

  // prv_lock();

  // if (ehcill_state == eHCILLStateAwake || ehcill_state == eHCILLStateWaitForAck) {
  //   PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "Already awake or waking up");
  //   prv_unlock();
  //   return;
  // }

  exti_disable(BT_GPIO_EXTI_CONFIG);
  uart_init(BLUETOOTH_UART);
  uart_set_rx_interrupt_enabled(BLUETOOTH_UART, true);
}

static bool requested_wakeup = false;

static void ble_chipset_go_to_sleep(void) {
  prv_lock();

  PBL_ASSERTN(ehcill_state != eHCILLStateAsleep);

  // requested sleep while we're waiting for wakeup!
  if (ehcill_state == eHCILLStateWaitForAck) {
    BREAKPOINT;
    prv_unlock();
    return;
  }
  
  uart_set_tx_interrupt_enabled(BLUETOOTH_UART, false);
  uart_wait_for_tx_complete(BLUETOOTH_UART);

  // update the state machine
  ehcill_state = eHCILLStateAsleep;

  // pull RTS high
  uart_assert_rts(BLUETOOTH_UART);

  // enable wakeup via CTS
  exti_enable(BT_GPIO_EXTI_CONFIG);

  // send sleep ACK
  uart_write_byte(BLUETOOTH_UART, HCILL_GO_TO_SLEEP_ACK);
  uart_wait_for_tx_complete(BLUETOOTH_UART);

  // turn off the UART
  uart_deinit(BLUETOOTH_UART);

  PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "Entered sleep");

  bool did_request_wakeup = requested_wakeup;
  if (requested_wakeup) {
    // If a wakeup was requested while sleeping, we need to handle it
    requested_wakeup = false;
  }

  prv_unlock();
  
  if (did_request_wakeup) {
    // If we requested a wakeup, we need to wake up the chipset
    PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "Waking up chipset after sleep request");
    ble_chipset_ensure_awake();
  }

  // stop_mode_enable(InhibitorBluetooth);
}

bool ble_chipset_ensure_awake(void) {
  prv_lock();
  if (ehcill_state == eHCILLStateAwake) {
    prv_unlock();
    return true;
  }

  requested_wakeup = true;

  if (ehcill_state == eHCILLStateWaitForAck) {
    prv_unlock();
    return false;
  }

  PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "Waking up chipset");

  // disable the wakeup interrupt
  exti_disable(BT_GPIO_EXTI_CONFIG);

  ehcill_state = eHCILLStateWaitForAck;

  uart_init(BLUETOOTH_UART);
  uart_set_rx_interrupt_enabled(BLUETOOTH_UART, true);
  uart_write_byte(BLUETOOTH_UART, HCILL_WAKE_UP_IND);

  PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "Send HCILL_WAKE_UP_IND");

  prv_unlock();

  return false;
}

bool ble_chipset_handle_hci_frame(uint8_t pkt_type, void *data) {
  bool handled = false;

  switch (pkt_type) {
    case HCILL_GO_TO_SLEEP_IND:
      PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "HCILL_GO_TO_SLEEP_IND received");
      ble_chipset_go_to_sleep();
      handled = true;
      break;
    case HCILL_WAKE_UP_ACK:
      prv_lock();
      PBL_ASSERTN(ehcill_state == eHCILLStateWaitForAck);
      PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "HCILL_WAKE_UP_ACK received");
      prv_return_to_awake();
      prv_unlock();
      handled = true;
      break;
    case HCILL_WAKE_UP_IND:
      prv_lock();
      PBL_ASSERTN(ehcill_state == eHCILLStateAsleep || ehcill_state == eHCILLStateWaitForAck);
      PBL_LOG_D(LOG_DOMAIN_BT, LOG_LEVEL_INFO, "HCILL_WAKE_UP_IND received");

      // ACK the wake up indication
      uart_write_byte(BLUETOOTH_UART, HCILL_WAKE_UP_ACK);

      // maybe we shouldn't wait here?  
      uart_wait_for_tx_complete(BLUETOOTH_UART);

      prv_return_to_awake();
      prv_unlock();
      handled = true;
      break;
  }

  return handled;
}
