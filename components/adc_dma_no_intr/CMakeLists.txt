idf_component_register(SRCS "esp32_adc_dma_driver.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc  driver freertos) 