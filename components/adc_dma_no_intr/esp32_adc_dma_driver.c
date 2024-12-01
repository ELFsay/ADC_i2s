#include "esp32_adc_dma_driver.h"
#include <esp_heap_caps.h>
#include <hal/i2s_ll.h>
#include "esp_clk_tree.h"
#include <driver/gpio.h>
#include "soc/lldesc.h"
#include <esp_adc/adc_cali_scheme.h>
// 多adc单元不完整且未测试
//  #define DMA_CONTINUE false
#define DMA_INTR false
#define ADC_ATTEN_DB ADC_ATTEN_DB_12 // adc参考电压1.1v
adc_channel_t *adc_channel = NULL;
adc_unit_t *adc_unit_id = NULL;
uint32_t adc_conv_frame_size = 0;
bool conv_mode_single = true; // 是否为单通道模式

adc_continuous_handle_t adc_handle = NULL;
adc_cali_handle_t cali_handle_t_1 = NULL;
adc_cali_handle_t cali_handle_t_2 = NULL;

static const char *TAG = "adc_dma_oneshot";
#define INTERNAL_BUF_NUM 3
#if CONFIG_IDF_TARGET_ESP32
esp_err_t i2s_platform_acquire_occupation(int id, const char *comp_name);
void adc_apb_periph_claim(void);
#define adc_ll_digi_dma_enable() adc_ll_digi_set_data_source(1) // Will this influence I2S0
#define adc_ll_digi_dma_disable() adc_ll_digi_set_data_source(0)
/*
#define adc_dma_ll_rx_get_intr(dev, mask) ({ i2s_ll_get_intr_status(dev) & mask; })
#define adc_dma_ll_rx_clear_intr(dev, chan, mask) i2s_ll_clear_intr_status(dev, mask)
#define adc_dma_ll_rx_enable_intr(dev, chan, mask) \
    do                                             \
    {                                              \
        ((i2s_dev_t *)(dev))->int_ena.val |= mask; \
    } while (0)
#define adc_dma_ll_rx_disable_intr(dev, chan, mask) \
    do                                              \
    {                                               \
        ((i2s_dev_t *)(dev))->int_ena.val &= ~mask; \
    } while (0)
#define adc_dma_ll_rx_reset_channel(dev, chan) i2s_ll_rx_reset_dma(dev)
#define adc_dma_ll_rx_stop(dev, chan) i2s_ll_rx_stop_link(dev)
#define adc_dma_ll_rx_start(dev, chan, address)                   \
    do                                                            \
    {                                                             \
        ((i2s_dev_t *)(dev))->in_link.addr = (uint32_t)(address); \
        i2s_ll_enable_dma(dev, 1);                                \
        ((i2s_dev_t *)(dev))->in_link.start = 1;                  \
    } while (0)
#define adc_dma_ll_get_in_suc_eof_desc_addr(dev, chan) ({uint32_t addr; i2s_ll_rx_get_eof_des_addr(dev, &addr); addr; })
#define adc_ll_digi_dma_set_eof_num(dev, num)   \
    do                                          \
    {                                           \
        ((i2s_dev_t *)(dev))->rx_eof_num = num; \
    } while (0)
#define adc_ll_digi_reset(dev)     \
    do                             \
    {                              \
        i2s_ll_rx_reset(dev);      \
        i2s_ll_rx_reset_fifo(dev); \
    } while (0)
#define adc_ll_digi_trigger_enable(dev) i2s_ll_rx_start(dev)
#define adc_ll_digi_trigger_disable(dev) i2s_ll_rx_stop(dev)
#define adc_ll_digi_dma_enable() adc_ll_digi_set_data_source(1) // Will this influence I2S0
#define adc_ll_digi_dma_disable() adc_ll_digi_set_data_source(0)
*/
#endif

IRAM_ATTR bool read_adc_data(int32_t *data, uint8_t *channel, uint32_t gpio_num)
{

    while (adc_hal_check_event(&adc_handle->hal, ADC_HAL_DMA_INTR_MASK) == false) // 等待DMA转换换成
        ;
    adc_hal_digi_clr_intr(&adc_handle->hal, ADC_HAL_DMA_INTR_MASK); // 清除DMA标志位

    // 不知道该不该加
    // adc_handle->rx_eof_desc_addr = adc_hal_get_desc_addr(&adc_handle->hal);
    //
    // adc_ll_digi_dma_disable();
    // adc_ll_digi_trigger_disable(adc_handle->hal.dev);

    // 转换数据格式
    // for (int i = 0; i < gpio_num; i++)
    for (int i = 0; i < adc_conv_frame_size * INTERNAL_BUF_NUM; i += SOC_ADC_DIGI_RESULT_BYTES)
    {
        adc_digi_output_data_t *p = (void *)&adc_handle->rx_dma_buf[i];
        // static int count = 0;//数据足够后跳出 但无法使用滤波器

        for (int j = 0; j < gpio_num; j++)
        {
            if (conv_mode_single == true)
            {
                if (adc_channel[j] == p->type1.channel)
                {
                    channel[j] = p->type1.channel;
                    // data[j] = p->type1.data;
                    // adc_cali_raw_to_voltage(cali_handle_t_1, p->type1.data, &data[j]);
                    if (adc_unit_id[j] == ADC_UNIT_1)
                        adc_cali_raw_to_voltage(cali_handle_t_1, p->type1.data, &data[j]);
                    else
                        adc_cali_raw_to_voltage(cali_handle_t_2, p->type1.data, &data[j]);
                    // count++;
                    // break;
                }
            }
            // else
            // {
            //     if (adc_channel[j] == p->type2.channel)
            //     {
            //         channel[j] = p->type2.channel;
            //         // data[j] = p->type2.data;
            //         if (adc_unit_id[j] == ADC_UNIT_1)
            //             adc_cali_raw_to_voltage(cali_handle_t_1, p->type2.data, &data[j]);
            //         else
            //             adc_cali_raw_to_voltage(cali_handle_t_2, p->type2.data, &data[j]);
            //         // count++;
            //         // break;
            //     }
            // }
            // printf("%d:%04ld\t", channel[j], data[j]);
        }
        // if (count == gpio_num)
        // {
        //     count = 0;
        //     break;
        // }
        // adc_cali_raw_to_voltage();
        // 此处可以加滤波器 如FIR滤波器
    }
    // printf("\t");

    // 不知道该不该加

    //  adc_ll_digi_dma_enable();
    //  adc_ll_digi_trigger_enable(adc_handle->hal.dev);

    return true;
}

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
static esp_err_t adc_dma_new_handle(const adc_continuous_handle_cfg_t *hdl_config, adc_continuous_handle_t *ret_handle)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE((hdl_config->conv_frame_size % SOC_ADC_DIGI_DATA_BYTES_PER_CONV == 0), ESP_ERR_INVALID_ARG, TAG, "conv_frame_size should be in multiples of `SOC_ADC_DIGI_DATA_BYTES_PER_CONV`");

    adc_continuous_ctx_t *adc_ctx = heap_caps_calloc(1, sizeof(adc_continuous_ctx_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (adc_ctx == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

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
#endif
    if (ret != ESP_OK)
    {
        goto cleanup;
    }
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
void adc_dma_init(uint8_t *gpio, uint8_t gpio_num, uint32_t sample_freq_hz)
{
    adc_channel = heap_caps_calloc(1, gpio_num * sizeof(adc_channel_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    adc_unit_id = heap_caps_calloc(1, gpio_num * sizeof(adc_unit_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    uint8_t channel_num = gpio_num;
    for (int i = 0; i < gpio_num; i++) // 获取ADC通道号和单元号
    {
        ESP_ERROR_CHECK(adc_continuous_io_to_channel(gpio[i], &adc_unit_id[i], &adc_channel[i]));
        if ((i > 0) && (adc_unit_id[i] != adc_unit_id[i - 1]))
        {
            conv_mode_single = false;
        }
    }
    // adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * INTERNAL_BUF_NUM,
        .conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV,
    };
    if (gpio_num % 2 == 1)
    {
        adc_config.conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * (gpio_num / 2 + 1);
    }
    else
        adc_config.conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV * (gpio_num / 2);
    adc_conv_frame_size = adc_config.conv_frame_size;

    ESP_ERROR_CHECK(adc_dma_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = sample_freq_hz,
    };
    if (conv_mode_single == true)
    {
        if (adc_unit_id[0] == ADC_UNIT_1)
            dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
        else
            dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_2;
    }
    else
    {
        dig_cfg.conv_mode = ADC_CONV_BOTH_UNIT;
        dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    }

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = ADC_ATTEN_DB;
        adc_pattern[i].channel = adc_channel[i];
        adc_pattern[i].unit = adc_unit_id[i];
        if (conv_mode_single == true)
            adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
        else
            adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH - 1;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config_1 = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_cali_line_fitting_config_t cali_config_2 = {
        .unit_id = ADC_UNIT_2,
        .atten = ADC_ATTEN_DB,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    if (conv_mode_single == false)
    {
        cali_config_1.bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH - 1;
        cali_config_2.bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH - 1;
    }

    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config_1, &cali_handle_t_1));
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config_2, &cali_handle_t_2));

    // *out_handle = handle;

    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}
// static void adc_hal_digi_dma_link_descriptors(dma_descriptor_t *desc, uint8_t *data_buf, uint32_t per_eof_size, uint32_t eof_step, uint32_t eof_num)
// {
//     HAL_ASSERT(((uint32_t)data_buf % 4) == 0);
//     HAL_ASSERT((per_eof_size % 4) == 0);
//     uint32_t n = 0;
//     dma_descriptor_t *desc_head = desc;

//     while (eof_num--)
//     {
//         uint32_t eof_size = per_eof_size;

//         for (int i = 0; i < eof_step; i++)
//         {
//             uint32_t this_len = eof_size;
//             if (this_len > DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED)
//             {
//                 this_len = DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED;
//             }

//             desc[n] = (dma_descriptor_t){
//                 .dw0.size = this_len,
//                 .dw0.length = 0,
//                 .dw0.suc_eof = 0,
//                 .dw0.owner = 1,
//                 .buffer = data_buf,
//                 .next = &desc[n + 1]};
//             eof_size -= this_len;
//             data_buf += this_len;
//             n++;
//         }
//     }

//     desc[n - 1].next = desc_head;
//     // desc[n - 1].dw0.suc_eof = 1;
//     // desc[n - 1].next = NULL;
// }

// void adc_dma_oneshot_start(adc_hal_dma_ctx_t *hal, uint8_t *data_buf)
// {
//     // stop peripheral and DMA
//     // adc_hal_digi_stop(hal);

//     // reset DMA
//     adc_dma_ll_rx_reset_channel(hal->dev, hal->dma_chan);
//     // reset peripheral
//     adc_ll_digi_reset(hal->dev);

//     // reset the current descriptor address
//     hal->cur_desc_ptr = &hal->desc_dummy_head;
//     adc_hal_digi_dma_link_descriptors(hal->rx_desc, data_buf, hal->eof_num * SOC_ADC_DIGI_DATA_BYTES_PER_CONV, hal->eof_step, hal->eof_desc_num);

//     // start DMA
//     adc_dma_ll_rx_start(hal->dev, hal->dma_chan, (lldesc_t *)hal->rx_desc);
//     // connect DMA and peripheral
//     adc_ll_digi_dma_enable();
//     // start ADC
//     adc_ll_digi_trigger_enable(hal->dev);
// }
