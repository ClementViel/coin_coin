
/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "driver/gpio.h"
#include "errno.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"

#include "app.h"
#include "hid.h"
#include "wav.h"

QueueHandle_t app_event_queue = NULL;
/**
 * @brief Scancode to ascii table
 */
const uint8_t keycode2ascii[57][2] = {
    {0, 0},     /* HID_KEY_NO_PRESS        */
    {0, 0},     /* HID_KEY_ROLLOVER        */
    {0, 0},     /* HID_KEY_POST_FAIL       */
    {0, 0},     /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'}, /* HID_KEY_A               */
    {'b', 'B'}, /* HID_KEY_B               */
    {'c', 'C'}, /* HID_KEY_C               */
    {'d', 'D'}, /* HID_KEY_D               */
    {'e', 'E'}, /* HID_KEY_E               */
    {'f', 'F'}, /* HID_KEY_F               */
    {'g', 'G'}, /* HID_KEY_G               */
    {'h', 'H'}, /* HID_KEY_H               */
    {'i', 'I'}, /* HID_KEY_I               */
    {'j', 'J'}, /* HID_KEY_J               */
    {'k', 'K'}, /* HID_KEY_K               */
    {'l', 'L'}, /* HID_KEY_L               */
    {'m', 'M'}, /* HID_KEY_M               */
    {'n', 'N'}, /* HID_KEY_N               */
    {'o', 'O'}, /* HID_KEY_O               */
    {'p', 'P'}, /* HID_KEY_P               */
    {'q', 'Q'}, /* HID_KEY_Q               */
    {'r', 'R'}, /* HID_KEY_R               */
    {'s', 'S'}, /* HID_KEY_S               */
    {'t', 'T'}, /* HID_KEY_T               */
    {'u', 'U'}, /* HID_KEY_U               */
    {'v', 'V'}, /* HID_KEY_V               */
    {'w', 'W'}, /* HID_KEY_W               */
    {'x', 'X'}, /* HID_KEY_X               */
    {'y', 'Y'}, /* HID_KEY_Y               */
    {'z', 'Z'}, /* HID_KEY_Z               */
    {'1', '!'}, /* HID_KEY_1               */
    {'2', '@'}, /* HID_KEY_2               */
    {'3', '#'}, /* HID_KEY_3               */
    {'4', '$'}, /* HID_KEY_4               */
    {'5', '%'}, /* HID_KEY_5               */
    {'6', '^'}, /* HID_KEY_6               */
    {'7', '&'}, /* HID_KEY_7               */
    {'8', '*'}, /* HID_KEY_8               */
    {'9', '('}, /* HID_KEY_9               */
    {'0', ')'}, /* HID_KEY_0               */
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /* HID_KEY_ENTER */
    {0, 0},      /* HID_KEY_ESC             */
    {'\b', 0},   /* HID_KEY_DEL             */
    {0, 0},      /* HID_KEY_TAB             */
    {' ', ' '},  /* HID_KEY_SPACE           */
    {'-', '_'},  /* HID_KEY_MINUS           */
    {'=', '+'},  /* HID_KEY_EQUAL           */
    {'[', '{'},  /* HID_KEY_OPEN_BRACKET    */
    {']', '}'},  /* HID_KEY_CLOSE_BRACKET   */
    {'\\', '|'}, /* HID_KEY_BACK_SLASH      */
    {'\\', '|'},
    /* HID_KEY_SHARP           */ // HOTFIX: for NonUS Keyboards repeat
                                  // HID_KEY_BACK_SLASH
    {';', ':'},                   /* HID_KEY_COLON           */
    {'\'', '"'},                  /* HID_KEY_QUOTE           */
    {'`', '~'},                   /* HID_KEY_TILDE           */
    {',', '<'},                   /* HID_KEY_LESS            */
    {'.', '>'},                   /* HID_KEY_GREATER         */
    {'/', '?'}                    /* HID_KEY_SLASH           */
};

/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char) {
  if (!!key_char) {
    putchar(key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
    if (KEYBOARD_ENTER_MAIN_CHAR == key_char) {
      putchar('\n');
    }
#endif // KEYBOARD_ENTER_LF_EXTEND
    fflush(stdout);
  }
}

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto) {
  static hid_protocol_t prev_proto_output = -1;

  if (prev_proto_output != proto) {
    prev_proto_output = proto;
    printf("\r\n");
    if (proto == HID_PROTOCOL_MOUSE) {
      printf("Mouse\r\n");
    } else if (proto == HID_PROTOCOL_KEYBOARD) {
      printf("Keyboard\r\n");
    } else {
      printf("Generic\r\n");
    }
    fflush(stdout);
  }
}

/**
 * @brief HID Keyboard modifier verification for capitalization application
 * (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier) {
  if (((modifier & HID_LEFT_SHIFT) == HID_LEFT_SHIFT) ||
      ((modifier & HID_RIGHT_SHIFT) == HID_RIGHT_SHIFT)) {
    return true;
  }
  return false;
}

#if KEYBOARD_ENTER_ALT_ESCAPE
/**
 * @brief HID Keyboard modifier verification for capitalization application
 * (right or left alt)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right alt)
 * @return false Modifier was not pressed (left or right alt)
 *
 */
static inline bool hid_keyboard_is_modifier_alt(uint8_t modifier) {
  if (((modifier & HID_LEFT_ALT) == HID_LEFT_ALT) ||
      ((modifier & HID_RIGHT_ALT) == HID_RIGHT_ALT)) {
    return true;
  }
  return false;
}

/**
 * @brief HID Keyboard alt code process(Called when ALT is pressed)
 *
 * @param[in] key_code Entered key value
 * @return true  Key values that qualify for ALT escape processing
 * @return false Key values that do not comply with ALT escape processing
 *
 */
static inline bool hid_keyboard_alt_code_processing(uint8_t key_code) {
  if ((key_code < HID_KEY_KEYPAD_1) || (key_code > HID_KEY_KEYPAD_0)) {
    return false;
  }
  if (key_code == HID_KEY_KEYPAD_0) {
    if (alt_code == 0) {
      is_ansi = true;
      return true;
    }
    /* Note: Since the keyboard code 0 of the numeric keypad is not keyboard
     * code 1 minus 1, the conversion is performed here to facilitate subsequent
     * calculations of the input numbers.
     */
    key_code = HID_KEY_KEYPAD_1 - 1;
  }
  alt_code = alt_code * 10 + (key_code - (HID_KEY_KEYPAD_1 - 1));
  return true;
}

/**
 * @brief HID Keyboard alt code process complete(Called when ALT is not pressed)
 */
static inline void hid_keyboard_alt_code_process_complete(void) {
  if (alt_code > 0) {
    alt_code = alt_code & 0xff;
    if (is_ansi || alt_code == 0) {
      char utf8_buffer[8] = {0};
      if (alt_code == 0) {
        alt_code = 0x100;
      }
      // ANSI is processed as UTF8
      if (alt_code <= 0x7F) {
        utf8_buffer[0] = (char)alt_code;
      } else {
        utf8_buffer[0] = 0xC0 | ((alt_code >> 6) & 0x1F);
        utf8_buffer[1] = 0x80 | (alt_code & 0x3F);
      }
      printf("%s", utf8_buffer);
      fflush(stdout);
    } else {
      hid_keyboard_print_char(alt_code);
    }
    alt_code = 0;
  }
  is_ansi = false;
}
#endif

/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier, uint8_t key_code,
                                         unsigned char *key_char) {
  uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;

#if KEYBOARD_ENTER_ALT_ESCAPE
  if (hid_keyboard_is_modifier_alt(modifier)) {
    // ALT modifier is still pressed
    if (hid_keyboard_alt_code_processing(key_code)) {
      // ALT code processed, no need to go further
      return false;
    }
  }
#endif

  if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_SLASH)) {
    *key_char = keycode2ascii[key_code][mod];
  } else {
    // All other key pressed
    return false;
  }

  return true;
}

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event) {
  unsigned char key_char;

  hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);

  if (KEY_STATE_PRESSED == key_event->state) {
    if (hid_keyboard_get_char(key_event->modifier, key_event->key_code,
                              &key_char)) {
      hid_keyboard_print_char(key_char);
      ESP_LOGI(TAG, "COIN ?");

      play_wav(MOUNT_POINT "/quack2.wav");
    }
  }
}

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src, uint8_t key,
                             unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    if (src[i] == key) {
      return true;
    }
  }
  return false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data,
                                              const int length) {
  hid_keyboard_input_report_boot_t *kb_report =
      (hid_keyboard_input_report_boot_t *)data;

  if (length < sizeof(hid_keyboard_input_report_boot_t)) {
    return;
  }

  static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
  key_event_t key_event;

#if KEYBOARD_ENTER_ALT_ESCAPE
  if (!hid_keyboard_is_modifier_alt(kb_report->modifier.val)) {
    hid_keyboard_alt_code_process_complete();
  }
#endif

  for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {

    // key has been released verification
    if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
        !key_found(kb_report->key, prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
      key_event.key_code = prev_keys[i];
      key_event.modifier = 0;
      key_event.state = KEY_STATE_RELEASED;
      key_event_callback(&key_event);
    }

    // key has been pressed verification
    if (kb_report->key[i] > HID_KEY_ERROR_UNDEFINED &&
        !key_found(prev_keys, kb_report->key[i], HID_KEYBOARD_KEY_MAX)) {
      key_event.key_code = kb_report->key[i];
      key_event.modifier = kb_report->modifier.val;
      key_event.state = KEY_STATE_PRESSED;
      key_event_callback(&key_event);
    }
  }

  memcpy(prev_keys, &kb_report->key, HID_KEYBOARD_KEY_MAX);
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data,
                                             const int length) {
  hid_print_new_device_report_header(HID_PROTOCOL_NONE);
  for (int i = 0; i < length; i++) {
    printf("%02X", data[i]);
  }
  putchar('\r');
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg) {
  uint8_t data[64] = {0};
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(
        hid_device_handle, data, 64, &data_length));

    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
      if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
        hid_host_keyboard_report_callback(data, data_length);
      }
    } else {
      hid_host_generic_report_callback(data, data_length);
    }

    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "HID Device, protocol '%s' DISCONNECTED",
             hid_proto_name_str[dev_params.proto]);
    ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    ESP_LOGI(TAG, "HID Device, protocol '%s' TRANSFER_ERROR",
             hid_proto_name_str[dev_params.proto]);
    break;
  default:
    ESP_LOGW(TAG,
             "HID Device, protocol '%s' Unhandled event: %d (possibly "
             "suspend/resume)",
             hid_proto_name_str[dev_params.proto], event);
    break;
  }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event, void *arg) {
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_DRIVER_EVENT_CONNECTED:
    ESP_LOGI(TAG, "HID Device, protocol '%s' CONNECTED",
             hid_proto_name_str[dev_params.proto]);

    const hid_host_device_config_t dev_config = {
        .callback = hid_host_interface_callback, .callback_arg = NULL};

    if (dev_params.proto != HID_PROTOCOL_NONE) {
      ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
      if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
        ESP_ERROR_CHECK(hid_class_request_set_protocol(
            hid_device_handle, HID_REPORT_PROTOCOL_BOOT));
        if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
          ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle, 0, 0));
        }
      }
      ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
    }
    break;
  default:
    break;
  }
}

/**
 * @brief Start USB Host install and handle common USB host library events while
 * app pin not low
 *
 * @param[in] arg  Not used
 */
void usb_lib_task(void *arg) {
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LOWMED,
  };

  ESP_ERROR_CHECK(usb_host_install(&host_config));
  xTaskNotifyGive(arg);

  while (true) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    // In this example, there is only one client registered
    // So, once we deregister the client, this call must succeed with ESP_OK
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      ESP_ERROR_CHECK(usb_host_device_free_all());
      break;
    }
  }

  ESP_LOGI(TAG, "USB shutdown");
  // Clean up USB Host
  vTaskDelay(10); // Short delay to allow clients clean-up
  ESP_ERROR_CHECK(usb_host_uninstall());
  vTaskDelete(NULL);
}

/**
 * @brief BOOT button pressed callback
 *
 * Signal application to exit the HID Host task
 *
 * @param[in] arg Unused
 */
void gpio_isr_cb(void *arg) {
  BaseType_t xTaskWoken = pdFALSE;

  const app_event_queue_t evt_queue = {
      .event_group = APP_EVENT,
  };

  if (app_event_queue) {
    xQueueSendFromISR(app_event_queue, &evt_queue, &xTaskWoken);
  }

  if (xTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event, void *arg) {
  const app_event_queue_t evt_queue = {.event_group = APP_EVENT_HID_HOST,
                                       // HID Host Device related info
                                       .hid_host_device.handle =
                                           hid_device_handle,
                                       .hid_host_device.event = event,
                                       .hid_host_device.arg = arg};

  if (app_event_queue) {
    xQueueSend(app_event_queue, &evt_queue, 0);
  }
}
