
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

/* GPIO Pin number for quit from example logic */
#define APP_QUIT_PIN GPIO_NUM_0

extern QueueHandle_t app_event_queue;

/**
 * @brief APP event group
 *
 * Application logic can be different. There is a one among other ways to
 * distinguish the event by application event group. In this example we have two
 * event groups: APP_EVENT            - General event, which is APP_QUIT_PIN
 * press event (Generally, it is IO0). APP_EVENT_HID_HOST   - HID Host Driver
 * event, such as device connection/disconnection or input report.
 */
typedef enum { APP_EVENT = 0, APP_EVENT_HID_HOST } app_event_group_t;

/**
 * @brief APP event queue
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct {
  app_event_group_t event_group;
  /* HID Host - Device related info */
  struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t event;
    void *arg;
  } hid_host_device;
} app_event_queue_t;

/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = {"NONE", "KEYBOARD", "MOUSE"};

/**
 * @brief Key event
 */
typedef struct {
  enum key_state { KEY_STATE_PRESSED = 0x00, KEY_STATE_RELEASED = 0x01 } state;
  uint8_t modifier;
  uint8_t key_code;
} key_event_t;

/* Main char symbol for ENTER key */
#define KEYBOARD_ENTER_MAIN_CHAR '\r'
/* When set to 1 pressing ENTER will be extending with LineFeed during serial
 * debug output */
#define KEYBOARD_ENTER_LF_EXTEND 1
/* When set to 1, numbers entered from the numeric keypad while ALT is pressed
 * will be escaped */
#define KEYBOARD_ENTER_ALT_ESCAPE 1

#if KEYBOARD_ENTER_ALT_ESCAPE
static bool is_ansi = false;
static unsigned int alt_code = 0;
#endif


/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char);

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto);

/**
 * @brief HID Keyboard modifier verification for capitalization application
 * (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier);

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
static inline bool hid_keyboard_is_modifier_alt(uint8_t modifier);

/**
 * @brief HID Keyboard alt code process(Called when ALT is pressed)
 *
 * @param[in] key_code Entered key value
 * @return true  Key values that qualify for ALT escape processing
 * @return false Key values that do not comply with ALT escape processing
 *
 */
static inline bool hid_keyboard_alt_code_processing(uint8_t key_code);

/**
 * @brief HID Keyboard alt code process complete(Called when ALT is not pressed)
 */
static inline void hid_keyboard_alt_code_process_complete(void);
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
                                         unsigned char *key_char);

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event);

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src, uint8_t key,
                             unsigned int length);

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data,
                                              const int length);

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data,
                                             const int length);
/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg);
/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event, void *arg);

/**
 * @brief Start USB Host install and handle common USB host library events while
 * app pin not low
 *
 * @param[in] arg  Not used
 */
void usb_lib_task(void *arg);

/**
 * @brief BOOT button pressed callback
 *
 * Signal application to exit the HID Host task
 *
 * @param[in] arg Unused
 */
void gpio_isr_cb(void *arg);
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
                              const hid_host_driver_event_t event, void *arg);
