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
#include "esp32_adc_i2s_driver.h"

// #include <C:\Espressif\frameworks\esp-idf-v5.1.4\components\esp_adc\adc_continuous_internal.h>
#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN 256

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[2] = {ADC_CHANNEL_6, ADC_CHANNEL_7};
#else
static adc_channel_t channel[2] = {ADC_CHANNEL_2, ADC_CHANNEL_3};
#endif
adc_continuous_handle_t handle = NULL;
esp_err_t ret;
uint32_t ret_num = 0;
uint8_t result[EXAMPLE_READ_LEN] = {0};
static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";
int ADC_count = 0;
static uint32_t IRAM_ATTR i2s_adc_buffer[2] = {0};

static void IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    // BaseType_t mustYield = pdFALSE;
    // // Notify that ADC continuous driver has done enough number of conversions
    // vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    // return (mustYield == pdTRUE);
    ADC_count++;
    // return true;
}
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
    for (uint32_t i = 0; i < times; i++)
    {
        // x*=3.5f;//120ns
        // size_t size = 0;
        uint8_t data_buf[10] = {0};
        uint32_t size = 0;
        read_adc_data(handle, data_buf, &size);
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
    // printf("ADC_count:%d ,Cycles_us:%f ,KHz:%f %ld\t", ADC_count, total_time_us / ADC_count, 1000.0f * ADC_count / total_time_us, GET_PERI_REG_MASK(I2S_INT_RAW_REG(0), I2S_RX_REMPTY_INT_RAW_M));
    // ADC_count = 0;
    // printf("ADC chan_num:%ld data %ld\t", chan_num, data);
    printf("  total time:%f us, average time %f ns  KHz:%f \n", total_time_us, avg_time_us * 1000.0f, 1000.0f / avg_time_us);
    esp_rom_delay_us(1000);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4,
        .conv_frame_size = 4,
    };
    // ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
    ESP_ERROR_CHECK(adc_i2s_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 200 * 1000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void app_main(void)
{
    esp_task_wdt_deinit();

    memset(result, 0xcc, EXAMPLE_READ_LEN);

    // s_task_handle = xTaskGetCurrentTaskHandle();

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb, // 3130ns
        // .on_conv_done = NULL//224ns

    };
    // ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1)
    {

        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

        while (1)
        {
            Hz1();
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

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}
