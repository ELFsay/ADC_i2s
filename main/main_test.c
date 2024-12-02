/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include <soc/soc.h>
#include <soc/i2s_reg.h>
#include <soc/i2s_struct.h>
#include "esp32_adc_dma_driver.h"

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";
int ADC_count = 0;
static uint32_t IRAM_ATTR i2s_adc_buffer[2] = {0};
#define FQ_Hz 1500 * 1000
// #define COV_PER_PIN 4
#define gpio_num 3
#define speed_test 1 // 0 启用vofa波形读取
uint8_t adc_io[gpio_num] = {34, 35, 32, 33};
 adc_atten_t atten= ADC_ATTEN_DB_12;
void Hz1()
{
    volatile float float_a = 1.0f;
    volatile int int_a = 1;
    int times = 1000;

    uint32_t for_start = esp_timer_get_time();
    for (uint32_t i = 0; i < times; i++)
        ;
    uint32_t for_end = esp_timer_get_time();
    float for_avg_time = (for_end - for_start) * 1.0f / times;
    // printf("for 1M times, total time:%ld us, average time%f \t", for_end - for_start, for_avg_time);
    // delay(200);
    volatile float x = 1.3f;

    uint32_t start = esp_timer_get_time();
    ADC_count = 0;

    uint32_t chan_num = 0;
    uint32_t data = 0;
    // size_t size = 0;
    int32_t data_buf[10] = {0};
    uint8_t channel[10] = {0};
    // read_adc_data(data_buf, channel, gpio_num);

    for (uint32_t i = 0; i < times; i++)
    {
        // x*=3.5f;//120ns
//  readFiFo();
       if( read_adc_data(data_buf, channel, gpio_num)==false)
       printf("read_adc_data error\n");
//    if( read_adc_data(data_buf, channel, gpio_num)==false)
//        printf("read_adc_data error\n");
//          if( read_adc_data(data_buf, channel, gpio_num)==false)
//        printf("read_adc_data error\n");
        // adc_hal_digi_start(&handle->hal, handle->rx_dma_buf);
        // adc_hal_digi_stop(&handle->hal);
        //  adc_dma_ll_rx_stop(handle->hal.dev, handle->hal.dma_chan);
        //  adc_dma_ll_rx_start(handle->hal.dev, handle->hal.dma_chan, handle->hal.rx_desc);
        // read_adc_data(data_buf, channel, gpio_num);
        // while (adc_hal_check_event(&handle->hal, ADC_HAL_DMA_INTR_MASK) == false)
        //     ;
        // adc_hal_digi_clr_intr(&handle->hal, ADC_HAL_DMA_INTR_MASK);

        // // while (xRingbufferReceiveUpTo(handle->ringbuf_hdl, &size, 1, 4)==NULL)
        // //     ;
        // while (adc_continuous_read(handle, result, 4, &ret_num, 0) != ESP_OK)
        //     ;
        // adc_continuous_read(handle, result, 4, &ret_num, 0);
        // adc_digi_output_data_t *p = (void *)&result[0];
        // chan_num = p->type1.channel;
        // data = p->type1.data;
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

    // memset(result, 0xcc, EXAMPLE_READ_LEN);

    // s_task_handle = xTaskGetCurrentTaskHandle();

    // continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);
    adc_dma_init(adc_io, sizeof(adc_io) / sizeof(uint8_t),atten, FQ_Hz);
    // adc_continuous_evt_cbs_t cbs = {
    //     .on_conv_done = s_conv_done_cb, // 3130ns
    //     // .on_conv_done = NULL//224ns

    // };
    // ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    // ESP_ERROR_CHECK(adc_continuous_start(handle));

    uint32_t chan_num = 0;
    uint32_t data = 0;
    // size_t size = 0;
    int32_t data_buf[10] = {0};
    uint8_t channel[10] = {0};
    while (1)
    {
#if speed_test == 1
        Hz1();
#else
        read_adc_data(data_buf, channel, gpio_num);
        printf("%06.1f,%06.1f\n", (float)data_buf[0], (float)data_buf[1]);
#endif
        // if (ret == ESP_OK) {
        //     ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
        //     for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
        //         adc_digi_output_data_t *p = (void*)&result[i];
        //         uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
        //         uint32_t data = EXAMPLE_ADC_GET_DATA(p);
        //         /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
        //         if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
        //             ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
        //         } else {
        //             ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
        //         }
        //     }
        //     /**
        //      * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
        //      * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
        //      * usually you don't need this delay (as this task will block for a while).
        //      */
        //     vTaskDelay(1);
        // } else if (ret == ESP_ERR_TIMEOUT) {
        //     //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
        //     break;
        // }
    }
}
