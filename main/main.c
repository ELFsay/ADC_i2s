// #include <string.h>
// #include <stdio.h>
// #include "sdkconfig.h"
// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"
// #include "esp_adc/adc_continuous.h"
// #include "esp_timer.h"
// #include "esp_task_wdt.h"
// #include <soc/soc.h>
// #include <soc/i2s_reg.h>
// #include <soc/i2s_struct.h>
// #include "esp32_i2s_driver.h"
// int ADC_count = 0;
// void Hz()
// {
//     volatile float float_a = 1.0f;
//     volatile int int_a = 1;
//     int times = 10000;

//     uint32_t for_start = esp_timer_get_time();
//     for (uint32_t i = 0; i < times; i++)
//         ;
//     uint32_t for_end = esp_timer_get_time();
//     float for_avg_time = (for_end - for_start) * 1.0f / times;
//     // printf("for 1M times, total time:%ld us, average time%f \t", for_end - for_start, for_avg_time);
//     // delay(200);
//     volatile float x = 1.3f;

//     uint32_t start = esp_timer_get_time();
//     ADC_count = 0;

//     for (uint32_t i = 0; i < times; i++)
//     {
//        readFiFo();
     
//     }
//     uint32_t end = esp_timer_get_time();
//     float total_time_us = (end - start) - (for_end - for_start);
//     float avg_time_us = (end - start) * 1.0f / times - for_avg_time;
//     printf("ADC_count:%d ,Cycles_us:%f ,KHz:%f %ld\t", ADC_count, total_time_us / ADC_count, 1000.0f * ADC_count / total_time_us, GET_PERI_REG_MASK(I2S_INT_RAW_REG(0), I2S_RX_REMPTY_INT_RAW_M));
//     ADC_count = 0;
//     printf("  total time:%f us, average time %f ns  KHz:%f \n", total_time_us, avg_time_us * 1000.0f, 1000.0f / avg_time_us);
//     esp_rom_delay_us(1000);
// }

// void app_main2(void)
// {
//     esp_task_wdt_deinit();
//     _configureI2S(0, 34, 35, 33);
//     while (1)
//     {
//         Hz();
//     }
// }