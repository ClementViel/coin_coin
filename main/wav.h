#include "esp_log.h"
#include "file_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm_audio.h"
#include <dirent.h>
#include <inttypes.h>
#include <stdio.h>
#include <sys/stat.h>

typedef struct {
  // The "RIFF" chunk descriptor
  uint8_t ChunkID[4];
  int32_t ChunkSize;
  uint8_t Format[4];
  // The "fmt" sub-chunk
  uint8_t Subchunk1ID[4];
  int32_t Subchunk1Size;
  int16_t AudioFormat;
  int16_t NumChannels;
  int32_t SampleRate;
  int32_t ByteRate;
  int16_t BlockAlign;
  int16_t BitsPerSample;
  // The "data" sub-chunk
  uint8_t Subchunk2ID[4];
  int32_t Subchunk2Size;
} wav_header_t;

#ifdef CONFIG_STORAGE_SDCARD
static char **g_file_list = NULL;
static uint16_t g_file_num = 0;
#endif

#ifdef CONFIG_IDF_TARGET_ESP32
#define GPIO_AUDIO_OUTPUT_L 25
#define GPIO_AUDIO_OUTPUT_R 26
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define GPIO_AUDIO_OUTPUT_L 1
#define GPIO_AUDIO_OUTPUT_R 2
#elif defined CONFIG_IDF_TARGET_ESP32S3
#define GPIO_AUDIO_OUTPUT_L 1
#define GPIO_AUDIO_OUTPUT_R 2
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define GPIO_AUDIO_OUTPUT_L 3
#define GPIO_AUDIO_OUTPUT_R 2
#else
#define GPIO_AUDIO_OUTPUT_L 1
#define GPIO_AUDIO_OUTPUT_R 2
#endif

esp_err_t play_wav(const char *filepath);
