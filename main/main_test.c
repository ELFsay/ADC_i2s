/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp32_adc_dma_driver.h"

#define FQ_Hz 1500 * 1000
#define gpio_num 3
#define speed_test 1 // 0 启用vofa波形读取
uint8_t adc_io[gpio_num] = {34, 35, 32, 33};

adc_atten_t atten = ADC_ATTEN_DB_12;
int32_t data_buf[10] = {0};
uint8_t channel[10] = {0};
void Hz1()
{

    int times = 1000;

    uint32_t for_start = esp_timer_get_time();
    for (uint32_t i = 0; i < times; i++)
        ;
    uint32_t for_end = esp_timer_get_time();
    float for_avg_time = (for_end - for_start) * 1.0f / times;
    // printf("for 1M times, total time:%ld us, average time%f \t", for_end - for_start, for_avg_time);

    uint32_t start = esp_timer_get_time();

    for (uint32_t i = 0; i < times; i++)
    {

        if (read_adc_data(data_buf, channel, gpio_num) == false)
            printf("read_adc_data error\n");

        for (int i = 0; i < gpio_num; i++)
        {
            float alph = 0.99f;
            static int32_t last_data[gpio_num] = {0};
            data_buf[i] = (data_buf[i] * alph + last_data[i] * (1.0f - alph));
            last_data[i] = data_buf[i];
        }
    }
    uint32_t end = esp_timer_get_time();
    float total_time_us = (end - start) - (for_end - for_start);
    float avg_time_us = (end - start) * 1.0f / times - for_avg_time;
    for (int i = 0; i < gpio_num; i++)
        printf("ADC chan_num %02d data %04ld\t", channel[i], data_buf[i]);
    // printf("\n");
    printf("  total time:%f us, average time %f ns  KHz:%f \n", total_time_us, avg_time_us * 1000.0f, 1000.0f / avg_time_us);
    esp_rom_delay_us(500);
}

void app_main(void)
{
    esp_task_wdt_deinit();
    adc_dma_init(adc_io, sizeof(adc_io) / sizeof(uint8_t), atten, FQ_Hz);
    while (1)
    {
#if speed_test == 1
        Hz1();
#else
        read_adc_data(data_buf, channel, gpio_num);
        printf("%06.1f,%06.1f\n", (float)data_buf[0], (float)data_buf[1]);
#endif
    }
}
