#ifndef esp32_adc_dma_driver_h_
#define esp32_adc_dma_driver_h_
#include <hal/adc_types.h>
#include <esp_adc/adc_filter.h>
#include <hal/adc_hal.h>
#include <driver/i2s_types.h>
#include <esp_intr_alloc.h>
#include <freertos/ringbuf.h>
#include <esp_pm.h>
#include "esp_check.h"
#include "esp_adc/adc_continuous.h" //其余函数可从该头文件中寻找
#include <esp_adc/adc_cali_scheme.h>

typedef enum
{
    ADC_FSM_INIT,
    ADC_FSM_STARTED,
} adc_fsm_t;

/*---------------------------------------------------------------
            Driver Context
---------------------------------------------------------------*/
typedef struct adc_iir_filter_t adc_iir_filter_t;
typedef struct adc_continuous_ctx_t adc_continuous_ctx_t;

/**
 * @brief ADC iir filter context
 */
struct adc_iir_filter_t
{
    adc_digi_iir_filter_t filter_id;        // Filter ID
    adc_continuous_iir_filter_config_t cfg; // filter configuration
    adc_continuous_ctx_t *continuous_ctx;   // ADC continuous driver context
};

/**
 * @brief ADC continuous driver context
 */
struct adc_continuous_ctx_t
{
    uint8_t *rx_dma_buf;   // dma buffer
    adc_hal_dma_ctx_t hal; // hal context
#if SOC_GDMA_SUPPORTED
    gdma_channel_handle_t rx_dma_channel; // dma rx channel handle
#elif CONFIG_IDF_TARGET_ESP32S2
    spi_host_device_t spi_host; // ADC uses this SPI DMA
#elif CONFIG_IDF_TARGET_ESP32
    i2s_port_t i2s_host; // ADC uses this I2S DMA
#endif
    intr_handle_t dma_intr_hdl;                  // DMA Interrupt handler
    RingbufHandle_t ringbuf_hdl;                 // RX ringbuffer handler
    void *ringbuf_storage;                       // Ringbuffer storage buffer
    void *ringbuf_struct;                        // Ringbuffer structure buffer
    intptr_t rx_eof_desc_addr;                   // eof descriptor address of RX channel
    adc_fsm_t fsm;                               // ADC continuous mode driver internal states
    bool use_adc1;                               // 1: ADC unit1 will be used; 0: ADC unit1 won't be used.
    bool use_adc2;                               // 1: ADC unit2 will be used; 0: ADC unit2 won't be used. This determines whether to acquire sar_adc2_mutex lock or not.
    adc_atten_t adc1_atten;                      // Attenuation for ADC1. On this chip each ADC can only support one attenuation.
    adc_atten_t adc2_atten;                      // Attenuation for ADC2. On this chip each ADC can only support one attenuation.
    adc_hal_digi_ctrlr_cfg_t hal_digi_ctrlr_cfg; // Hal digital controller configuration
    adc_continuous_evt_cbs_t cbs;                // Callbacks
    void *user_data;                             // User context
    esp_pm_lock_handle_t pm_lock;                // For power management
#if SOC_ADC_DIG_IIR_FILTER_SUPPORTED
    adc_iir_filter_t *iir_filter[SOC_ADC_DIGI_IIR_FILTER_NUM]; // ADC IIR filter context
#endif
};
typedef struct adc_handle_t
{

    uint8_t *gpio;
    adc_channel_t *adc_channel;
    adc_unit_t *adc_unit_id;
    uint8_t channel_num;

    adc_continuous_handle_t continuous_handle;
    adc_continuous_handle_cfg_t adc_config;
    adc_continuous_config_t dig_cfg;
    adc_atten_t atten[2];
    adc_cali_handle_t cali_handle_t[2];
} adc_handle_t;

// 读取ADC数据的函数。目前只支持单个单元的多通道读取。
// 参数：
// - data：指向存储ADC读取结果的uint32_t数组的指针。
// - channel：指向用于指定读取的ADC通道的uint8_t指针。
// - gpio_num：指定连接到ADC的GPIO引脚的数量。
bool read_adc_data(int32_t *data, uint8_t *channel, uint32_t gpio_num);

// esp_err_t adc_dma_new_handle(const adc_continuous_handle_cfg_t *hdl_config, adc_continuous_handle_t *ret_handle);

// 初始化ADC DMA。
// 参数：
// - gpio：指向GPIO数组的指针，用于指定ADC输入的GPIO引脚。
// - gpio_num：指定GPIO引脚的数量。
// - atten：指向ADC的衰减值数组的指针。
// - sample_freq_hz：采样频率（以赫兹为单位）。

void adc_dma_init(uint8_t *gpio, uint8_t gpio_num, adc_atten_t atten, uint32_t sample_freq_hz);


inline esp_err_t adc_dma_start(adc_continuous_handle_t handle) { adc_continuous_start(handle); }

inline esp_err_t adc_dma_stop(adc_continuous_handle_t handle) { adc_continuous_stop(handle); }

#endif //__ESP32_ADC_I2S_DRIVER_H__