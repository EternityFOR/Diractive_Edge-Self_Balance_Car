/**
 * 
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 *
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */
#pragma once

#include "driver/gpio.h"
#include "esp_idf_version.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev_os.h"

/**
 * @brief I2C GPIO
 */
#define FUNC_I2C_EN     (1)
#define I2C_NUM         (0)
#define I2C_CLK         (600000)
#define GPIO_I2C_SCL    (GPIO_NUM_18)
#define GPIO_I2C_SDA    (GPIO_NUM_8)

/**
 * @brief I2S GPIO for speaker
 */
#define I2S_SPEAKER_NUM         (I2S_NUM_0)
#define GPIO_I2S_SCLK           (GPIO_NUM_38)
#define GPIO_I2S_LRCK           (GPIO_NUM_39)
#define GPIO_I2S_DOUT           (GPIO_NUM_20)
#define GPIO_I2S_MCLK           (GPIO_NUM_2)

/**
 * @brief I2S GPIO for microphone
 */
#define I2S_MIC_NUM             (I2S_NUM_1)
#define GPIO_I2S_SDIN           (GPIO_NUM_16)
#define GPIO_I2S_SCLK_MIC       (GPIO_NUM_15)
#define GPIO_I2S_LRCK_MIC       (GPIO_NUM_7)
#define I2S_MIC_MCLK_MULTIPLE   384

/**
 * @brief 
 * 
 */
#define FUNC_PWR_CTRL       (1)
#define GPIO_PWR_CTRL       (GPIO_NUM_46)
#define GPIO_PWR_ON_LEVEL   (1)

/**
 * @brief Default configurations for I2S
 */
#define I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan) { \
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate), \
    .clk_cfg.mclk_multiple = I2S_MIC_MCLK_MULTIPLE,\
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits_per_chan, I2S_SLOT_MODE_STEREO), \
    .gpio_cfg = { \
        .bclk = GPIO_I2S_SCLK_MIC, \
        .ws   = GPIO_I2S_LRCK, \
        .dout = GPIO_I2S_DOUT, \
        .din  = GPIO_I2S_SDIN, \
        .invert_flags = { \
            .mclk_inv = false,\
            .bclk_inv = false,\
            .ws_inv = false,\
        },\
    }, \
}
