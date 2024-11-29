#include "esp32_adc_i2s_driver.h"
#include <esp_heap_caps.h>
#define RING_BUFFFER false
#define DMA_INTR false

static const char *ADC_TAG = "adc_i2s";
#define INTERNAL_BUF_NUM 5
esp_err_t i2s_platform_acquire_occupation(int id, const char *comp_name);
void adc_apb_periph_claim(void);

IRAM_ATTR bool read_adc_data(adc_continuous_ctx_t *adc_digi_ctx, uint8_t *data, uint32_t *size)
{

    while (adc_hal_check_event(&adc_digi_ctx->hal, ADC_HAL_DMA_INTR_MASK) == false) // 等待DMA转换换成
        ;
    adc_hal_digi_clr_intr(&adc_digi_ctx->hal, ADC_HAL_DMA_INTR_MASK); // 清除DMA标志位
    adc_digi_ctx->rx_eof_desc_addr = adc_hal_get_desc_addr(&adc_digi_ctx->hal);

    while (1) // 读取ADC数据
    {
        adc_hal_dma_desc_status_t status = adc_hal_get_reading_result(&adc_digi_ctx->hal, adc_digi_ctx->rx_eof_desc_addr, &data, size);
        if (status != ADC_HAL_DMA_DESC_VALID) // 读取完毕
        {
            return true;
        }
        //转换数据格式
        for (int i = 0; i < *size; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (void *)&data[i];
            uint32_t chan_num = p->type1.channel;
            uint32_t data = p->type1.data;
            // printf("ADC chan_num %ld data %ld\t", chan_num, data);
            }
    }
    return false;
}
// IRAM_ATTR bool get_adc_value(uint8_t *data, uint32_t *size)
// {

//     for (int i = 0; i < *size; i += SOC_ADC_DIGI_RESULT_BYTES)
//     {
//         adc_digi_output_data_t *p = (void *)&data[i];
//         uint32_t chan_num = p->type1.channel;
//         uint32_t data = p->type1.data;
//         // printf("ADC chan_num %ld data %ld\t", chan_num, data);
//     }
//     // printf("\n\n\n\n\n\n");
//     return true;
// }
IRAM_ATTR bool s_adc_dma_intr(adc_continuous_ctx_t *adc_digi_ctx)
{
    portBASE_TYPE taskAwoken = 0;
    bool need_yield = false;
    BaseType_t ret;
    adc_hal_dma_desc_status_t status = false;
    uint8_t *finished_buffer = NULL;
    uint32_t finished_size = 0;

    while (1)
    {
        status = adc_hal_get_reading_result(&adc_digi_ctx->hal, adc_digi_ctx->rx_eof_desc_addr, &finished_buffer, &finished_size);
        if (status != ADC_HAL_DMA_DESC_VALID)
        {
            break;
        }
#if RING_BUFFFER == true

        ret = xRingbufferSendFromISR(adc_digi_ctx->ringbuf_hdl, finished_buffer, finished_size, &taskAwoken);
        need_yield |= (taskAwoken == pdTRUE);
#endif
        if (adc_digi_ctx->cbs.on_conv_done) // 用户注册的回调函数
        {
            adc_continuous_evt_data_t edata = {
                .conv_frame_buffer = finished_buffer,
                .size = finished_size,
            };
            if (adc_digi_ctx->cbs.on_conv_done(adc_digi_ctx, &edata, adc_digi_ctx->user_data))
            {
                need_yield |= true;
            }
        }
#if RING_BUFFFER == true

        if (ret == pdFALSE)
        {
            // ringbuffer overflow
            if (adc_digi_ctx->cbs.on_pool_ovf)
            {
                adc_continuous_evt_data_t edata = {};
                if (adc_digi_ctx->cbs.on_pool_ovf(adc_digi_ctx, &edata, adc_digi_ctx->user_data))
                {
                    need_yield |= true;
                }
            }
        }
#endif
    }

    return need_yield;
}
IRAM_ATTR void adc_dma_intr_handler(void *arg)
{
    adc_continuous_ctx_t *ctx = (adc_continuous_ctx_t *)arg;
    bool need_yield = false;

    bool conversion_finish = adc_hal_check_event(&ctx->hal, ADC_HAL_DMA_INTR_MASK);
    if (conversion_finish)
    {
        adc_hal_digi_clr_intr(&ctx->hal, ADC_HAL_DMA_INTR_MASK);

        intptr_t desc_addr = adc_hal_get_desc_addr(&ctx->hal);

        ctx->rx_eof_desc_addr = desc_addr;
        need_yield = s_adc_dma_intr(ctx);
    }

    if (need_yield)
    {
        portYIELD_FROM_ISR();
    }
}
esp_err_t adc_i2s_new_handle(const adc_continuous_handle_cfg_t *hdl_config, adc_continuous_handle_t *ret_handle)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE((hdl_config->conv_frame_size % SOC_ADC_DIGI_DATA_BYTES_PER_CONV == 0), ESP_ERR_INVALID_ARG, ADC_TAG, "conv_frame_size should be in multiples of `SOC_ADC_DIGI_DATA_BYTES_PER_CONV`");

    adc_continuous_ctx_t *adc_ctx = heap_caps_calloc(1, sizeof(adc_continuous_ctx_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (adc_ctx == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
#if RING_BUFFFER == true
    // ringbuffer storage/struct buffer
    adc_ctx->ringbuf_storage = heap_caps_calloc(1, hdl_config->max_store_buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    adc_ctx->ringbuf_struct = heap_caps_calloc(1, sizeof(StaticRingbuffer_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!adc_ctx->ringbuf_storage || !adc_ctx->ringbuf_struct)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    // ringbuffer
    adc_ctx->ringbuf_hdl = xRingbufferCreateStatic(hdl_config->max_store_buf_size, RINGBUF_TYPE_BYTEBUF, adc_ctx->ringbuf_storage, adc_ctx->ringbuf_struct);
    if (!adc_ctx->ringbuf_hdl)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
#endif
    // malloc internal buffer used by DMA
    adc_ctx->rx_dma_buf = heap_caps_calloc(1, hdl_config->conv_frame_size * INTERNAL_BUF_NUM, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!adc_ctx->rx_dma_buf)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    // malloc dma descriptor
    uint32_t dma_desc_num_per_frame = (hdl_config->conv_frame_size + DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED - 1) / DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED;
    uint32_t dma_desc_max_num = dma_desc_num_per_frame * INTERNAL_BUF_NUM;
    adc_ctx->hal.rx_desc = heap_caps_calloc(1, (sizeof(dma_descriptor_t)) * dma_desc_max_num, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!adc_ctx->hal.rx_desc)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    // malloc pattern table
    adc_ctx->hal_digi_ctrlr_cfg.adc_pattern = calloc(1, SOC_ADC_PATT_LEN_MAX * sizeof(adc_digi_pattern_config_t));
    if (!adc_ctx->hal_digi_ctrlr_cfg.adc_pattern)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

#if CONFIG_PM_ENABLE
    ret = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "adc_dma", &adc_ctx->pm_lock);
    if (ret != ESP_OK)
    {
        goto cleanup;
    }
#endif // CONFIG_PM_ENABLE

#if SOC_GDMA_SUPPORTED
    // alloc rx gdma channel
    gdma_channel_alloc_config_t rx_alloc_config = {
        .direction = GDMA_CHANNEL_DIRECTION_RX,
    };
    ret = gdma_new_channel(&rx_alloc_config, &adc_ctx->rx_dma_channel);
    if (ret != ESP_OK)
    {
        goto cleanup;
    }
    gdma_connect(adc_ctx->rx_dma_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_ADC, 0));

    gdma_strategy_config_t strategy_config = {
        .auto_update_desc = true,
        .owner_check = true};
    gdma_apply_strategy(adc_ctx->rx_dma_channel, &strategy_config);

    gdma_rx_event_callbacks_t cbs = {
        .on_recv_eof = adc_dma_in_suc_eof_callback};
    gdma_register_rx_event_callbacks(adc_ctx->rx_dma_channel, &cbs, adc_ctx);

    int dma_chan;
    gdma_get_channel_id(adc_ctx->rx_dma_channel, &dma_chan);

#elif CONFIG_IDF_TARGET_ESP32S2
    // ADC utilises SPI3 DMA on ESP32S2
    bool spi_success = false;
    uint32_t dma_chan = 0;

    spi_success = spicommon_periph_claim(SPI3_HOST, "adc");
    ret = spicommon_dma_chan_alloc(SPI3_HOST, SPI_DMA_CH_AUTO, &dma_chan, &dma_chan);
    if (ret == ESP_OK)
    {
        adc_ctx->spi_host = SPI3_HOST;
    }
    if (!spi_success || (adc_ctx->spi_host != SPI3_HOST))
    {
        goto cleanup;
    }

    ret = esp_intr_alloc(spicommon_irqdma_source_for_host(adc_ctx->spi_host), ESP_INTR_FLAG_IRAM, adc_dma_intr_handler,
                         (void *)adc_ctx, &adc_ctx->dma_intr_hdl);
    if (ret != ESP_OK)
    {
        goto cleanup;
    }

#elif CONFIG_IDF_TARGET_ESP32
    // ADC utilises I2S0 DMA on ESP32
    uint32_t dma_chan = 0;
    ret = i2s_platform_acquire_occupation(I2S_NUM_0, "adc");
    if (ret != ESP_OK)
    {
        ret = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }

    adc_ctx->i2s_host = I2S_NUM_0;
#if DMA_INTR == true
    ret = esp_intr_alloc(i2s_periph_signal[adc_ctx->i2s_host].irq, ESP_INTR_FLAG_IRAM, adc_dma_intr_handler,
                         (void *)adc_ctx, &adc_ctx->dma_intr_hdl);

    if (ret != ESP_OK)
    {
        goto cleanup;
    }
#endif
#endif

    adc_hal_dma_config_t config = {
#if SOC_GDMA_SUPPORTED
        .dev = (void *)GDMA_LL_GET_HW(0),
#elif CONFIG_IDF_TARGET_ESP32S2
        .dev = (void *)SPI_LL_GET_HW(adc_ctx->spi_host),
#elif CONFIG_IDF_TARGET_ESP32
        .dev = (void *)I2S_LL_GET_HW(adc_ctx->i2s_host),
#endif
        .eof_desc_num = INTERNAL_BUF_NUM,
        .eof_step = dma_desc_num_per_frame,
        .dma_chan = dma_chan,
        .eof_num = hdl_config->conv_frame_size / SOC_ADC_DIGI_DATA_BYTES_PER_CONV};
    adc_hal_dma_ctx_config(&adc_ctx->hal, &config);

    adc_ctx->fsm = ADC_FSM_INIT;
    *ret_handle = adc_ctx;

    adc_apb_periph_claim();

#if SOC_ADC_CALIBRATION_V1_SUPPORTED
    adc_hal_calibration_init(ADC_UNIT_1);
    adc_hal_calibration_init(ADC_UNIT_2);
#endif // #if SOC_ADC_CALIBRATION_V1_SUPPORTED

    return ret;

cleanup:
    adc_continuous_deinit(adc_ctx);
    return ret;
}