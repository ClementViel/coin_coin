idf_component_register(SRCS "hid_host_example.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_driver_gptimer fatfs spiffs vfs
                       )
if(NOT CONFIG_STORAGE_SDCARD)
    spiffs_create_partition_image(audio ${CMAKE_CURRENT_SOURCE_DIR}/../spiffs_image FLASH_IN_PROJECT)
endif()
