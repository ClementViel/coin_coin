#include <dirent.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "errno.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"
#include "usb/usb_host.h"

#include "app.h"
#include "driver/gpio.h"
#include "file_manager.h"
#include "hid.h"
#include "wav.h"

/* GPIO Pin number for quit from example logic */
#define APP_QUIT_PIN GPIO_NUM_0

#ifdef CONFIG_STORAGE_SDCARD
static char **g_file_list = NULL;
static uint16_t g_file_num = 0;
#endif

#define GPIO_AUDIO_OUTPUT_L 1
#define GPIO_AUDIO_OUTPUT_R 2

void app_main(void) {
  BaseType_t task_created;
  app_event_queue_t evt_queue;
  esp_err_t ret;

  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes",
           esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
  ESP_LOGI(TAG, "       ⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⡶⠿⠿⠷⣶⣄⠀⠀⠀⠀⠀");
  ESP_LOGI(TAG, "       ⠀⠀⠀⠀⠀⠀⠀⠀⣰⡿⠁⠀⠀⢀⣀⡀⠙⣷⡀⠀⠀⠀");
  ESP_LOGI(TAG, "      ⠀⡀⠀⠀⠀⠀⠀⢠⣿⠁⠀⠀⠀⠘⠿⠃⠀⢸⣿⣿⣿⣿ ");
  ESP_LOGI(TAG, "     ⣠⡿⠛⢷⣦⡀⠀⠀⠈⣿⡄⠀⠀⠀⠀⠀⠀⠀⣸⣿⣿⣿⠟ ");
  ESP_LOGI(TAG, "    ⢰⡿⠁⠀⠀⠙⢿⣦⣤⣤⣼⣿⣄⠀⠀⠀⠀⠀⢴⡟⠛⠋⠁⠀ ");
  ESP_LOGI(TAG, "    ⣿⠇⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠁⠀⠀⠀⠀⠀⠈⣿⡀⠀⠀⠀ ");
  ESP_LOGI(TAG, "    ⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⡇⠀⠀⠀ ");
  ESP_LOGI(TAG, "    ⣿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⡇⠀⠀⠀ ");
  ESP_LOGI(TAG, "    ⣷⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡿⠀⠀⠀⠀  ");
  ESP_LOGI(TAG, "    ⠹⣷⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣰⡿⠁⠀⠀⠀⠀   ");
  ESP_LOGI(TAG, "      ⠉⠙⠛⠿⠶⣶⣶⣶⣶⣶⠶⠿⠟⠛⠉⠀⠀⠀⠀⠀⠀   ");

  pwm_audio_config_t pac = {
      .duty_resolution = LEDC_TIMER_10_BIT,
      .gpio_num_left = GPIO_AUDIO_OUTPUT_L,
      .ledc_channel_left = LEDC_CHANNEL_0,
      .gpio_num_right = GPIO_AUDIO_OUTPUT_R,
      .ledc_channel_right = LEDC_CHANNEL_1,
      .ledc_timer_sel = LEDC_TIMER_0,
      .ringbuf_len = 1024 * 8,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
      .tg_num = TIMER_GROUP_0,
      .timer_num = TIMER_0,
#endif
  };
  const gpio_config_t input_pin = {
      .pin_bit_mask = BIT64(APP_QUIT_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };

  // Init audio stack
  ret = pwm_audio_init(&pac);

  if (ESP_OK != ret) {
    ESP_LOGE(TAG, "error initializing audio stack");
    return;
  }

  // Init file system
  ret = fm_spiffs_init();
  if (ESP_OK != ret) {
    ESP_LOGE(TAG, "sdcard initial failed, exit");
    return;
  }
  // Init BOOT button: Pressing the button simulates app request to exit
  // It will disconnect the USB device and uninstall the HID driver and USB Host
  // Lib
  ESP_ERROR_CHECK(gpio_config(&input_pin));
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LOWMED));
  ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_isr_cb, NULL));

  /*
   * Create usb_lib_task to:
   * - initialize USB Host library
   * - Handle USB Host events while APP pin in in HIGH state
   */
  task_created =
      xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                              xTaskGetCurrentTaskHandle(), 2, NULL, 0);
  assert(task_created == pdTRUE);

  // Wait for notification from usb_lib_task to proceed
  ulTaskNotifyTake(false, 1000);

  /*
   * HID host driver configuration
   * - create background task for handling low level event inside the HID driver
   * - provide the device callback to get new HID Device connection event
   */
  const hid_host_driver_config_t hid_host_driver_config = {
      .create_background_task = true,
      .task_priority = 5,
      .stack_size = 4096,
      .core_id = 0,
      .callback = hid_host_device_callback,
      .callback_arg = NULL};

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

  // Create queue
  app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));

  ESP_LOGI(TAG, "Waiting for HID Device to be connected");

  while (1) {
    // Wait queue
    if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY)) {
      if (APP_EVENT == evt_queue.event_group) {
        // User pressed button
        usb_host_lib_info_t lib_info;
        ESP_ERROR_CHECK(usb_host_lib_info(&lib_info));
        if (lib_info.num_devices == 0) {
          // End while cycle
          break;
        } else {
          ESP_LOGW(TAG, "To shutdown example, remove all USB devices and press "
                        "button again.");
          // Keep polling
        }
      }

      if (APP_EVENT_HID_HOST == evt_queue.event_group) {
        hid_host_device_event(evt_queue.hid_host_device.handle,
                              evt_queue.hid_host_device.event,
                              evt_queue.hid_host_device.arg);
      }
    }
  }
}
