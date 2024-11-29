#include <adc_types.h>
#include <adc_filter.h>
#include <adc_hal.h>
#include <driver/i2s_types.h>
#include <esp_intr_alloc.h>
#include <ringbuf.h>
#include <esp_pm.h>

typedef enum {
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
struct adc_iir_filter_t {
    adc_digi_iir_filter_t filter_id;                            // Filter ID
    adc_continuous_iir_filter_config_t  cfg;                    //filter configuration
    adc_continuous_ctx_t                *continuous_ctx;        //ADC continuous driver context
};

/**
 * @brief ADC continuous driver context
 */
struct adc_continuous_ctx_t {
    uint8_t                         *rx_dma_buf;                //dma buffer
    adc_hal_dma_ctx_t               hal;                        //hal context
#if SOC_GDMA_SUPPORTED
    gdma_channel_handle_t           rx_dma_channel;             //dma rx channel handle
#elif CONFIG_IDF_TARGET_ESP32S2
    spi_host_device_t               spi_host;                   //ADC uses this SPI DMA
#elif CONFIG_IDF_TARGET_ESP32
    i2s_port_t                      i2s_host;                   //ADC uses this I2S DMA
#endif
    intr_handle_t                   dma_intr_hdl;               //DMA Interrupt handler
    RingbufHandle_t                 ringbuf_hdl;                //RX ringbuffer handler
    void*                           ringbuf_storage;            //Ringbuffer storage buffer
    void*                           ringbuf_struct;             //Ringbuffer structure buffer
    intptr_t                        rx_eof_desc_addr;           //eof descriptor address of RX channel
    adc_fsm_t                       fsm;                        //ADC continuous mode driver internal states
    bool                            use_adc1;                   //1: ADC unit1 will be used; 0: ADC unit1 won't be used.
    bool                            use_adc2;                   //1: ADC unit2 will be used; 0: ADC unit2 won't be used. This determines whether to acquire sar_adc2_mutex lock or not.
    adc_atten_t                     adc1_atten;                 //Attenuation for ADC1. On this chip each ADC can only support one attenuation.
    adc_atten_t                     adc2_atten;                 //Attenuation for ADC2. On this chip each ADC can only support one attenuation.
    adc_hal_digi_ctrlr_cfg_t        hal_digi_ctrlr_cfg;         //Hal digital controller configuration
    adc_continuous_evt_cbs_t        cbs;                        //Callbacks
    void                            *user_data;                 //User context
    esp_pm_lock_handle_t            pm_lock;                    //For power management
#if SOC_ADC_DIG_IIR_FILTER_SUPPORTED
    adc_iir_filter_t                *iir_filter[SOC_ADC_DIGI_IIR_FILTER_NUM];  //ADC IIR filter context
#endif
};
