#include "wav.h"
#include "app.h"
#include "esp_log.h"
#include "file_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm_audio.h"
#include <dirent.h>
#include <inttypes.h>
#include <stdio.h>
#include <sys/stat.h>

#ifdef CONFIG_STORAGE_SDCARD
static char **g_file_list = NULL;
static uint16_t g_file_num = 0;
#endif

esp_err_t play_wav(const char *filepath) {
  FILE *fd = NULL;
  struct stat file_stat;

  if (stat(filepath, &file_stat) == -1) {
    ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "file stat info: %s (%ld bytes)...", filepath,
           file_stat.st_size);
  fd = fopen(filepath, "r");

  if (NULL == fd) {
    ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
    return ESP_FAIL;
  }
  const size_t chunk_size = 4096;
  uint8_t *buffer = malloc(chunk_size);

  if (NULL == buffer) {
    ESP_LOGE(TAG, "audio data buffer malloc failed");
    fclose(fd);
    return ESP_FAIL;
  }

  /**
   * read head of WAV file
   */
  wav_header_t wav_head;
  int len = fread(&wav_head, 1, sizeof(wav_header_t), fd);
  if (len <= 0) {
    ESP_LOGE(TAG, "Read wav header failed");
    fclose(fd);
    return ESP_FAIL;
  }
  if (NULL == strstr((char *)wav_head.Subchunk1ID, "fmt") &&
      NULL == strstr((char *)wav_head.Subchunk2ID, "data")) {
    ESP_LOGE(TAG, "Header of wav format error");
    fclose(fd);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "frame_rate= %" PRIi32 ", ch=%d, width=%d", wav_head.SampleRate,
           wav_head.NumChannels, wav_head.BitsPerSample);

  pwm_audio_set_param(wav_head.SampleRate, wav_head.BitsPerSample,
                      wav_head.NumChannels);
  pwm_audio_set_volume(255);
  pwm_audio_start();

  /**
   * read wave data of WAV file
   */
  size_t write_num = 0;
  size_t cnt;

  do {
    /* Read file in chunks into the scratch buffer */
    len = fread(buffer, 1, chunk_size, fd);
    if (len <= 0) {
      break;
    }
    pwm_audio_write(buffer, len, &cnt, 1000 / portTICK_PERIOD_MS);

    write_num += len;
  } while (1);

  pwm_audio_stop();
  fclose(fd);
  ESP_LOGI(TAG, "File reading complete, total: %d bytes", write_num);
  return ESP_OK;
}
