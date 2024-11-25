//***Head_Files***
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "ble_t.h"
#include "blufi_app.h"
#include "Data_handle.h"
#include "wifi.h"
#include "main.h"
#include <driver/i2s_std.h>
//***Parameters***
//Tags
#define MAINTAG "main"
#define I2STAG "I2S"
//UART pin assign
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)

/* I2S port and GPIOs */
static bool stop_music_flag = true;
static i2s_chan_handle_t tx_handle = NULL;
static i2s_chan_handle_t rx_handle = NULL;
#define EXAMPLE_NUM_CHANNELS 2
#define SLOT_MODE I2S_SLOT_MODE_STEREO
#define EXAMPLE_MCLK_MULTIPLE 384
#define EXAMPLE_SAMPLE_RATE 16000
#define EXAMPLE_BIT_RATE I2S_DATA_BIT_WIDTH_16BIT
#define I2S_NUM 0
#define I2S_WS_IO  GPIO_NUM_39
#define I2S_BCK_IO GPIO_NUM_38
#define I2S_DO_IO  GPIO_NUM_37
#define I2S_DI_IO  GPIO_NUM_NC
extern const uint8_t music_pcm_start[] asm("_binary_car_2_raw_start");
extern const uint8_t music_pcm_end[]   asm("_binary_car_2_raw_end");
static TaskHandle_t i2s_music_task_handle = NULL;  // Task handle for the music playback
static bool is_music_playing = false;  // Flag to track if music is playing

//Queue
QueueHandle_t msg_queue = NULL;

//Data type

//***Functions***

// Initializzation
//uart init
void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 4096*5, 0, 0, NULL, 0);
}

//queue init
void queue_init(){
    msg_queue = xQueueCreate(50, sizeof(msg_struct));
    if (msg_queue == NULL) {
        ESP_LOGE(MAINTAG, "Failed to create queue");
    }
}

//i2s music
static esp_err_t i2s_driver_init(void) {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;  // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(EXAMPLE_BIT_RATE,
                                                      SLOT_MODE),
      .gpio_cfg =
          {
              .bclk = I2S_BCK_IO,
              .ws = I2S_WS_IO,
              .dout = I2S_DO_IO,
              .din = I2S_DI_IO,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };
  std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
  return ESP_OK;
}


static void i2s_music(void *args)
{
    esp_err_t ret = ESP_OK;
    size_t bytes_write = 0;
    uint8_t *data_ptr;
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    while (1) {
        if (stop_music_flag) {
            // 停止播放逻辑
            // ESP_LOGI(I2STAG, "[music] Music playback stopped.");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        // Reset data pointer to the start of the music data
        data_ptr = (uint8_t *)music_pcm_start;

        /* Preload the data before writing to the TX channel */
        ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
        ESP_ERROR_CHECK(i2s_channel_preload_data(tx_handle, data_ptr, music_pcm_end - data_ptr, &bytes_write));
        ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
        data_ptr += bytes_write;

        /* Write music to earphone */
        ret = i2s_channel_write(tx_handle, data_ptr, music_pcm_end - data_ptr, &bytes_write, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(I2STAG, "[music] i2s write failed, error code: %d", ret);
            abort();
        }

        if (bytes_write > 0) {
            ESP_LOGI(I2STAG, "[music] i2s music played, %d bytes are written.", bytes_write);
        } else {
            ESP_LOGE(I2STAG, "[music] i2s music play failed.");
            abort();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void start_music_playback() {
    if (!is_music_playing) {
        ESP_LOGI(MAINTAG, "Starting engine sound playback.");
        is_music_playing = true;
        stop_music_flag = false; // 清除停止标志
        ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
        ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    }
}

void stop_music_playback() {
    if (is_music_playing) {
        ESP_LOGI(MAINTAG, "Stopping engine sound playback.");
        is_music_playing = false;
        stop_music_flag = true; // 设置停止标志
        ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
        ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));  // 确保I2S通道被禁用
    }
}


// Main function to handle data and trigger music playback
void send_msg_to_stm32(void *pvParameters) {
    
    msg_struct msg_data;
    while (1) {
        if (xQueueReceive(msg_queue, &msg_data, portMAX_DELAY)) {
            char uart_data[2048];

            switch (msg_data.type) {
                case DataType_Control:
                    snprintf(uart_data, sizeof(uart_data), "{\"Type\":\"Control\",\"L\":%d,\"R\":%d,\"A\":%d}\n",
                             msg_data.leftWheelSpeed,
                             msg_data.rightWheelSpeed,
                             msg_data.action);

                    // Check wheel speeds to control engine sound playback
                    if ((msg_data.leftWheelSpeed != 0) || (msg_data.rightWheelSpeed != 0)) {
                        start_music_playback();  // Start music playback if any wheel is moving
                    } else {
                        stop_music_playback();  // Stop music playback if both wheels stop
                    }
                    break;

                case DataType_Weather:
                    snprintf(uart_data, sizeof(uart_data), "{\"Type\":\"Weather\",\"Data\":\"%s\"}\n",
                             msg_data.weatherData);
                    break;

                case DataType_PID:
                    snprintf(uart_data, sizeof(uart_data), "{\"Type\":\"PID\",\"Balance_Kp\":%f,\"Balance_Ki\":%f,\"Balance_Kd\":%f,\"Velocity_Kp\":%f,\"Velocity_Ki\":%f,\"Velocity_Kd\":%f}\n",
                            msg_data.balancePID.Kp,
                            msg_data.balancePID.Ki,
                            msg_data.balancePID.Kd,
                            msg_data.velocityPID.Kp,
                            msg_data.velocityPID.Ki,
                            msg_data.velocityPID.Kd);
                    break;

                case DataType_Mode_Control:
                    snprintf(uart_data, sizeof(uart_data), "{\"Type\":\"Mode_Control\"}\n");
                    break;

                default:
                    ESP_LOGE(MAINTAG, "Unknown data type received");
                    continue;  // Skip the uart_write_bytes if the type is unknown
            }

            uart_write_bytes(UART_NUM_1, uart_data, strlen(uart_data));
            ESP_LOGI(MAINTAG, "Sent data to UART: %s", uart_data);
        }
    }
}

//Main function
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Initialize WIFI
    initialise_wifi();

    // Start getting time and weather
    // weather_time_task_init();
    
    //Queue init
    queue_init();
    
    // Intialize UART
    init_uart();

    //Initialize Blutooth
    ble_init();

    //Task init
    xTaskCreate(send_msg_to_stm32, "send_msg_to_stm32", 4096*20, NULL, 10, NULL);

    //I2S init
    i2s_driver_init();
    xTaskCreate(i2s_music, "i2s_music", 4096 * 5, NULL, 5, &i2s_music_task_handle);
}
