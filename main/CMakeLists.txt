idf_component_register(SRCS "file_manager.c" "hid.c" "wav.c" "app_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_driver_gptimer fatfs spiffs vfs
                       )
if(NOT CONFIG_STORAGE_SDCARD)
    spiffs_create_partition_image(audio ./../spiffs_image FLASH_IN_PROJECT)
endif()
