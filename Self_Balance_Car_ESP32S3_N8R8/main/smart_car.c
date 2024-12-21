//***Head_Files***//
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
#include "smart_car.h"
#include <driver/i2s_std.h>
#include "lwip/sockets.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_crt_bundle.h"
#include "../managed_components/espressif__mdns/include/mdns.h"
#include "web_server.h"
#include "dns_server.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "model_path.h"
#include "esp_board_init.h"
#include "esp_process_sdkconfig.h"
#include "speech_commands_action.h"
//***Parameters***//
//Enable
#define WIFI_STA_ENABLE         1
#define WIFI_AP_ENABLE          0
#define OTA_ENABLE              0

//Tags
#define TAGMAIN      "main"
#define TAGSPEAKER   "I2S_SPEAKER"
#define TAGMIC       "I2S_MIC"
#define TAGTCP       "TCP"
#define TAGNVS       "NVS"
#define TAGOTA       "OTA"
#define WIFIPROVTAG  "WIFIPROV"
#define TAGAP        "WIFI_SoftAP"
#define TAGSTA       "WIFI_STA"

// UART pin assign
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)

/* I2S Speaker Configuration */
#define I2S_SPEAKER_NUM I2S_NUM_0
#define I2S_SPEAKER_SAMPLE_RATE 16000
#define I2S_SPEAKER_SLOT_MODE I2S_SLOT_MODE_STEREO
#define I2S_SPEAKER_MCLK_MULTIPLE 384
#define I2S_SPEAKER_BIT_WIDTH I2S_DATA_BIT_WIDTH_16BIT
#define I2S_SPEAKER_WS_IO  GPIO_NUM_39
#define I2S_SPEAKER_BCK_IO GPIO_NUM_38
#define I2S_SPEAKER_DO_IO  GPIO_NUM_37

/* I2S Microphone Configuration */
#define I2S_MIC_NUM I2S_NUM_1
#define I2S_MIC_SAMPLE_RATE 16000
#define I2S_MIC_SLOT_MODE I2S_SLOT_MODE_MONO
#define I2S_MIC_MCLK_MULTIPLE 384
#define I2S_MIC_BIT_WIDTH I2S_DATA_BIT_WIDTH_24BIT
#define I2S_MIC_WS_IO GPIO_NUM_7
#define I2S_MIC_BCK_IO GPIO_NUM_15
#define I2S_MIC_DI_IO GPIO_NUM_16

/* I2S Voice Recognition*/
int detect_flag = 0;
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
static srmodel_list_t *models = NULL;
static int play_voice = -2;

/***WIFI Configuration***/
static const char *WIFIPROV = "WIFIPROV";
#define AP_WIFI_SSID      "Smart_Car"
#define AP_WIFI_PASS      ""
#define AP_WIFI_CHANNEL             1
#define AP_MAX_User_RETRY           20
#define AP_MAX_RETRY_Internal       10
#define AP_MAX_RETRY_Manual         10
EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define USER_INPUT_BIT     BIT2
static int s_retry_num = 0;
extern wifi_config_t sta_config;
static uint8_t webserver_init = false;
/***WIFI Configuration***/

/*OTA defines*/
#define OTA_URL ""
#define OTA_RECV_TIMEOUT 5000

//***Sensors Status***//
sensor_status_t sensor_status = {
    .touch0_triggered = false,
    .touch1_triggered = false,
    .hs0_triggered    = false,
    .hs1_triggered    = false,
    .hs2_triggered    = false,
    .lidar_triggered  = false,
    .rfid_triggered   = false,
    .ads_triggered    = false,
};
//***Sensors Status***//
//***Adjustable Parameters***//
adjustable_params_t params = {
    .button_debounce           = 2,   //100ms
    .command_resent_times      = 2,
    .motor_pwm                 = 255, //0-255
    .rgb_pwm                   = 255, //0-255
    .motor_restart_delay       = 100, //5s
    .hall_sensor_debounce      = 5,   //250ms
    .home_found_dump_duration  = 200, //10s
    .home_found_reset_duration = 200, //10s
    .home_found_delay          = 100, //5s
    .h_r_angle_stop_delay      = 40   //2s
};
//***Adjustable Parameters***//

/* External audio data */
extern const uint8_t music_pcm_start[] asm("_binary_car_2_raw_start");
extern const uint8_t music_pcm_end[] asm("_binary_car_2_raw_end");

// I2S handles
static i2s_chan_handle_t i2s_speaker_handle = NULL;
static i2s_chan_handle_t i2s_mic_handle = NULL;

// Task handles
static TaskHandle_t i2s_speaker_task_handle = NULL;
static TaskHandle_t i2s_mic_task_handle = NULL;
static TaskHandle_t send_msg_to_stm32_task_handle = NULL;
static TaskHandle_t tcp_server_task_handle = NULL;

// Queue handle
QueueHandle_t msg_queue = NULL;

// Playback control
static bool is_music_playing = false;
static bool stop_music_flag = true;

// TCP server parameters
#define TCP_PORT 8080
#define LOG_BUFFER_SIZE 512
static char log_buffer[LOG_BUFFER_SIZE];
static int tcp_client_socket = -1;
static int tcp_server_socket = -1;

//Voice Command
typedef enum{
    GO = 40,
    STOP,
    RIGHT,
    LEFT,
    BACK,
};

//***Functions***//
/*NVS*/
static void nvs_init_main(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        ESP_LOGI(TAGNVS, "NVS flash erased");
    }
    ESP_ERROR_CHECK(ret);
}
void save_params_to_nvs(adjustable_params_t params) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAGNVS, "Failed to open NVS handle!");
        return;
    }

    #define SAVE_PARAM(key, value) \
        if ((err = nvs_set_i32(nvs_handle, key, value)) != ESP_OK) { \
            ESP_LOGE(TAGNVS, "Failed to save %s!", key); \
        }

    SAVE_PARAM("button_debounce", params.button_debounce);
    SAVE_PARAM("command_resent", params.command_resent_times);
    SAVE_PARAM("motor_pwm", params.motor_pwm);
    SAVE_PARAM("rgb_pwm", params.rgb_pwm);
    SAVE_PARAM("motor_restart", params.motor_restart_delay);
    SAVE_PARAM("hall_sensor", params.hall_sensor_debounce);
    SAVE_PARAM("home_dump", params.home_found_dump_duration);
    SAVE_PARAM("home_reset", params.home_found_reset_duration);
    SAVE_PARAM("home_delay", params.home_found_delay);
    SAVE_PARAM("angle_stop", params.h_r_angle_stop_delay);

    #undef SAVE_PARAM

    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    ESP_LOGI(TAGNVS, "Parameters saved to NVS.");
}
void load_params_from_nvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAGNVS, "Failed to open NVS handle!");
        return;
    }

    nvs_get_i32(nvs_handle, "button_debounce", &params.button_debounce);
    nvs_get_i32(nvs_handle, "command_resent", &params.command_resent_times);
    nvs_get_i32(nvs_handle, "motor_pwm", &params.motor_pwm);
    nvs_get_i32(nvs_handle, "rgb_pwm", &params.rgb_pwm);
    nvs_get_i32(nvs_handle, "motor_restart", &params.motor_restart_delay);
    nvs_get_i32(nvs_handle, "hall_sensor", &params.hall_sensor_debounce);
    nvs_get_i32(nvs_handle, "home_dump", &params.home_found_dump_duration);
    nvs_get_i32(nvs_handle, "home_reset", &params.home_found_reset_duration);
    nvs_get_i32(nvs_handle, "home_delay", &params.home_found_delay);
    nvs_get_i32(nvs_handle, "angle_stop", &params.h_r_angle_stop_delay);

    nvs_close(nvs_handle);
    ESP_LOGI(TAGNVS, "Parameters loaded from NVS.");
}
/*WIFI*/
/*OTA UPDATE*/
esp_err_t perform_ota_update(void)
{
    esp_http_client_config_t config = {
        .url = OTA_URL,
        // .cert_pem = (char *)server_cert_pem_start,
        .timeout_ms = OTA_RECV_TIMEOUT,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAGOTA, "OTA update successful. Rebooting...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        ESP_LOGE(TAGOTA, "OTA update failed. Error: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    return ESP_OK;
}
void save_wifi_credentials(const char *ssid, const char *password) {
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("wifi_config", NVS_READWRITE, &nvs_handle));

    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "ssid", ssid));
    ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "password", password));
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);

    ESP_LOGI(WIFIPROVTAG, "Wi-Fi credentials saved to NVS");
}
esp_netif_t *wifi_init_softap(void);
esp_netif_t *wifi_init_sta(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(WIFIPROVTAG, "WiFi event: %s, event_id=%ld", event_base, event_id);
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAGAP, "Station "MACSTR" joined, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAGAP, "Station "MACSTR" left, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAGSTA, "Station started");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAGSTA, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAGSTA, "Disconnected from AP, attempting reconnect...");
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
}
esp_netif_t *wifi_init_softap(void)
{
    esp_netif_t *esp_netif_ap = esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_WIFI_SSID,
            .password = AP_WIFI_PASS,
            .ssid_len = strlen(AP_WIFI_SSID),
            .channel = AP_WIFI_CHANNEL,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .max_connection = AP_MAX_User_RETRY,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    if (strlen(AP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    #if !WIFI_STA_ENABLE
        ESP_ERROR_CHECK(esp_wifi_start());
    #endif
    if(!webserver_init){
        webserver_init = true;
        ESP_ERROR_CHECK(start_web_server());
        start_mdns_service();
        start_dns_server();
    }
    ESP_LOGI(WIFIPROV, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             AP_WIFI_SSID, AP_WIFI_PASS, AP_WIFI_CHANNEL);
    return esp_netif_ap;
}
esp_netif_t *wifi_init_sta(void)
{
    ESP_LOGI(WIFIPROVTAG, "STA_INIT");
    esp_netif_t *esp_netif_sta = esp_netif_create_default_wifi_sta();
    wifi_config_t wifi_sta_config = {};
    
    wifi_sta_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_sta_config.sta.failure_retry_cnt = AP_MAX_RETRY_Internal;

    memcpy(wifi_sta_config.sta.ssid, sta_config.sta.ssid, sizeof(sta_config.sta.ssid));
    memcpy(wifi_sta_config.sta.password, sta_config.sta.password, sizeof(sta_config.sta.password));
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(WIFIPROV, "wifi_init_sta finished.");
    EventBits_t bits;
    do {
        bits = xEventGroupWaitBits(s_wifi_event_group,
                                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | USER_INPUT_BIT,
                                pdFALSE,
                                pdFALSE,
                                portMAX_DELAY);
        ESP_LOGI(WIFIPROV, "event bits: 0x%lx", bits);

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(WIFIPROV, "Connected to AP SSID:%s password:%s", sta_config.sta.ssid, sta_config.sta.password);
            save_wifi_credentials((const char*)sta_config.sta.ssid, (const char*)sta_config.sta.password);

            if (webserver_init) {
                ESP_LOGI(WIFIPROV, "Stopping AP server");
                // esp_wifi_set_mode(WIFI_MODE_STA);
                webserver_init = false;
            }
            stop_mdns_service();
            #if OTA_ENABLE
                perform_ota_update();
            #endif
            break;
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(WIFIPROV, "Failed to connect to SSID:%s, password:%s", sta_config.sta.ssid, sta_config.sta.password);
            if(s_retry_num < AP_MAX_RETRY_Manual) {
                s_retry_num++;
                ESP_ERROR_CHECK(esp_wifi_stop());
                ESP_ERROR_CHECK(esp_wifi_start());
                ESP_LOGI(WIFIPROV, "Retrying connection... %d", s_retry_num);
            } else {
                ESP_LOGI(WIFIPROV, "Failed to connect to SSID:%s, password:%s", sta_config.sta.ssid, sta_config.sta.password);
            }
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
        } else if (bits & USER_INPUT_BIT) {
            ESP_LOGI(WIFIPROV, "User Input event, trying to connect SSID:%s password:%s", sta_config.sta.ssid, sta_config.sta.password);
            memcpy(wifi_sta_config.sta.ssid, sta_config.sta.ssid, sizeof(sta_config.sta.ssid));
            memcpy(wifi_sta_config.sta.password, sta_config.sta.password, sizeof(sta_config.sta.password));
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));
            ESP_ERROR_CHECK(esp_wifi_stop());
            ESP_ERROR_CHECK(esp_wifi_start());
            xEventGroupClearBits(s_wifi_event_group, USER_INPUT_BIT);
        } else {
            ESP_LOGE(WIFIPROV, "UNEXPECTED EVENT");
        }
    } while (!(bits & WIFI_CONNECTED_BIT) || (bits & WIFI_FAIL_BIT));
    return esp_netif_sta;
}
static void wifi_init_main(){
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    NULL,
                    NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP,
                    &wifi_event_handler,
                    NULL,
                    NULL));

    /*Initialize WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    nvs_handle_t nvs_handle;
    size_t ssid_len = sizeof(sta_config.sta.ssid);
    size_t password_len = sizeof(sta_config.sta.password);
    ESP_ERROR_CHECK(nvs_open("wifi_config", NVS_READWRITE, &nvs_handle));
    esp_err_t ssid_err =
        nvs_get_str(nvs_handle, "ssid", (char *)sta_config.sta.ssid, &ssid_len);
    esp_err_t password_err =
        nvs_get_str(nvs_handle, "password", (char *)sta_config.sta.password, &password_len);
    nvs_close(nvs_handle);
    ESP_LOGI(WIFIPROVTAG,"Wi-Fi credentials found, ssid = %s, password = %s, connecting to " "Wi-Fi...", sta_config.sta.ssid, sta_config.sta.password);
    #if WIFI_AP_ENABLE
    {
        esp_netif_t *esp_netif_ap = wifi_init_softap();
    }
    #endif
    #if WIFI_STA_ENABLE
    {
        esp_netif_t *esp_netif_sta = wifi_init_sta();
    }
    #endif
}

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
    // uart_driver_install(UART_NUM_0, 4096*5, 0, 0, NULL, 0);
}
int custom_log_vprintf(const char *fmt, va_list args) {
    int len = vsnprintf(log_buffer, LOG_BUFFER_SIZE, fmt, args);
    if (len > 0 && tcp_client_socket > 0) {
        int sent = send(tcp_client_socket, log_buffer, len, 0);
        if (sent < 0) {
            ESP_LOGE("LOG_TCP", "Failed to send log via TCP: errno %d", errno);
        }
    }
    return vprintf(fmt, args);
}
void setup_log_redirection() {
    esp_log_set_vprintf(custom_log_vprintf);
}
void tcp_server_task(void *pvParameters) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_server_socket < 0) {
        ESP_LOGE(TAGTCP, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(tcp_server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAGTCP, "Socket unable to bind: errno %d", errno);
        close(tcp_server_socket);
        vTaskDelete(NULL);
        return;
    }

    if (listen(tcp_server_socket, 1) < 0) {
        ESP_LOGE(TAGTCP, "Error occurred during listen: errno %d", errno);
        close(tcp_server_socket);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAGTCP, "Socket listening on port %d", TCP_PORT);

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAGTCP, "Waiting for a new client...");
        tcp_client_socket = accept(tcp_server_socket, (struct sockaddr *)&client_addr, &client_addr_len);
        if (tcp_client_socket < 0) {
            ESP_LOGE(TAGTCP, "Unable to accept connection: errno %d", errno);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAGTCP, "Client connected");

        while (1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            char recv_buf[64];
            int len = recv(tcp_client_socket, recv_buf, sizeof(recv_buf) - 1, 0);
            if (len <= 0) {
                ESP_LOGE(TAGTCP, "Client disconnected or error: errno %d", errno);
                close(tcp_client_socket);
                tcp_client_socket = -1;
                break;
            }

            recv_buf[len] = '\0';
            ESP_LOGI(TAGTCP, "Received data: %s", recv_buf);

            if (send(tcp_client_socket, recv_buf, len, 0) < 0) {
                ESP_LOGE(TAGTCP, "Error occurred during sending: errno %d", errno);
                close(tcp_client_socket);
                tcp_client_socket = -1;
                break;
            }
        }
    }

    close(tcp_server_socket);
    vTaskDelete(NULL);
}

static esp_err_t i2s_speaker_init() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_SPEAKER_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_speaker_handle, NULL));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SPEAKER_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_SPEAKER_BIT_WIDTH, I2S_SPEAKER_SLOT_MODE),
        .gpio_cfg = {
            .bclk = I2S_SPEAKER_BCK_IO,
            .ws = I2S_SPEAKER_WS_IO,
            .dout = I2S_SPEAKER_DO_IO,
            .din = GPIO_NUM_NC, // No input
            .invert_flags =
            {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_SPEAKER_MCLK_MULTIPLE;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_speaker_handle, &std_cfg));
    return ESP_OK;
}
static esp_err_t i2s_mic_init() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_MIC_NUM, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &i2s_mic_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_MIC_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_MIC_BIT_WIDTH, I2S_MIC_SLOT_MODE),
        .gpio_cfg = {
            .bclk = I2S_MIC_BCK_IO,
            .ws = I2S_MIC_WS_IO,
            .din = I2S_MIC_DI_IO,
            .dout = GPIO_NUM_NC, // No output
            .invert_flags =
            {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MIC_MCLK_MULTIPLE;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_mic_handle, &std_cfg));
    return ESP_OK;
}
static void i2s_speaker_task(void *pvParameters) {
    esp_err_t ret = ESP_OK;
    size_t bytes_write = 0;
    uint8_t *data_ptr;
    size_t bytes_written = 0;

    ESP_ERROR_CHECK(i2s_channel_enable(i2s_speaker_handle));
    while (1) {
        if (stop_music_flag) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        data_ptr = (uint8_t *)music_pcm_start;

        /* Preload data */
        ESP_ERROR_CHECK(i2s_channel_disable(i2s_speaker_handle));
        ESP_ERROR_CHECK(i2s_channel_preload_data(i2s_speaker_handle, data_ptr, music_pcm_end - data_ptr, &bytes_written));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_speaker_handle));
        data_ptr += bytes_write;
        ESP_ERROR_CHECK(i2s_channel_write(i2s_speaker_handle, data_ptr, music_pcm_end - data_ptr, &bytes_written, portMAX_DELAY));

        if (bytes_write > 0) {
            ESP_LOGI(TAGSPEAKER, "[music] i2s music played, %d bytes are written.", bytes_write);
        } else {
            ESP_LOGE(TAGSPEAKER, "[music] i2s music play failed.");
            abort();
        }
        ESP_LOGI(TAGSPEAKER, "Music played, %d bytes written.", bytes_written);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void start_music_playback() {
    if (!is_music_playing) {
        ESP_LOGI(TAGMAIN, "Starting engine sound playback.");
        is_music_playing = true;
        stop_music_flag = false; // 清除停止标志
        ESP_ERROR_CHECK(i2s_channel_disable(i2s_speaker_handle));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_speaker_handle));
    }
}
void stop_music_playback() {
    if (is_music_playing) {
        ESP_LOGI(TAGMAIN, "Stopping engine sound playback.");
        is_music_playing = false;
        stop_music_flag = true;
        ESP_ERROR_CHECK(i2s_channel_disable(i2s_speaker_handle));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_speaker_handle));
    }
}
static void i2s_mic_task(void *pvParameters) {
    // Buffer for raw samples
    int32_t raw_samples[256];
    int16_t processed_samples[256];
    size_t bytes_read = 0;

    ESP_ERROR_CHECK(i2s_channel_enable(i2s_mic_handle));

    while (1) {
        // Read data from I2S in chunks
        ESP_ERROR_CHECK(i2s_channel_read(i2s_mic_handle, (void *)raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY));

        int samples_read = bytes_read / sizeof(int32_t);
        
        for (int i = 0; i < samples_read; i++) {
            // Process the raw sample data (similar to the shift in your reference code)
            processed_samples[i] = (raw_samples[i] & 0xFFFFFFF0) >> 11;
        }

        // Example: Log processed samples (you can replace this with further processing)
        for (int i = 0; i < samples_read; i++) {
            ESP_LOGI(TAGMIC, "Processed Sample: %05X", processed_samples[i]);
        }

        // Add delay to mimic periodic task execution
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void play_music(void *arg)
{
    while (task_flag) {
        switch (play_voice) {
        case -2:
            vTaskDelay(10);
            break;
        case -1:
            wake_up_action();
            play_voice = -2;
            break;
        default:
            speech_commands_action(play_voice);
            play_voice = -2;
            break;
        }
    }
    vTaskDelete(NULL);
}
void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    assert(nch <= feed_channel);
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    while (task_flag) {
        esp_get_feed_data(false, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, i2s_buff);
    }
    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}
void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    printf("multinet:%s\n", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data); // Add speech commands from sdkconfig
    assert(mu_chunksize == afe_chunksize);
    //print active speech commands
    multinet->print_active_speech_commands(model_data);

    printf("------------detect start------------\n");
    printf("task_flag:%d\n", task_flag);
    while (task_flag) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }

        // if (res->wakeup_state == WAKENET_DETECTED) {
        //     printf("WAKEWORD DETECTED\n");
        //     multinet->clean(model_data);
        // } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
        //     play_voice = -1;
        //     detect_flag = 1;
        //     printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
        //     // afe_handle->disable_wakenet(afe_data);
        //     // afe_handle->disable_aec(afe_data);
        // }
        if (1) {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++) {
                    printf("TOP %d, command_id: %d, phrase_id: %d, string: %s, prob: %f\n", 
                           i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }

                msg_struct msg_data = {0};
                msg_data.type = DataType_Control;

                switch (mn_result->command_id[0]) {
                    case GO:
                        printf("Command: GO\n");
                        msg_data.leftWheelSpeed = 100;
                        msg_data.rightWheelSpeed = 100;
                        msg_data.action = 0;
                        xQueueSend(msg_queue, &msg_data, portMAX_DELAY);
                        break;

                    case STOP:
                        printf("Command: STOP\n");
                        msg_data.leftWheelSpeed = 0;
                        msg_data.rightWheelSpeed = 0;
                        msg_data.action = 0;
                        xQueueSend(msg_queue, &msg_data, portMAX_DELAY);
                        break;

                    case RIGHT:
                        printf("Command: TURN RIGHT\n");
                        msg_data.leftWheelSpeed = 100;
                        msg_data.rightWheelSpeed = -100;
                        msg_data.action = 0;
                        xQueueSend(msg_queue, &msg_data, portMAX_DELAY);
                        break;

                    case LEFT:
                        printf("Command: TURN LEFT\n");
                        msg_data.leftWheelSpeed = -100;
                        msg_data.rightWheelSpeed = 100;
                        msg_data.action = 0;
                        xQueueSend(msg_queue, &msg_data, portMAX_DELAY);
                        break;

                    case BACK:
                        printf("Command: BACK\n");
                        msg_data.leftWheelSpeed = -100;
                        msg_data.rightWheelSpeed = -100;
                        msg_data.action = 0;
                        xQueueSend(msg_queue, &msg_data, portMAX_DELAY);
                        break;

                    default:
                        printf("Unknown command\n");
                        break;
                }

                printf("-----------listening-----------\n");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                detect_flag = 0;
                printf("\n-----------awaits to be waken up-----------\n");
                continue;
            }
        }
    }

    if (model_data) {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
}

void initialize_voice_recognition() {
    models = esp_srmodel_init("model");
    ESP_ERROR_CHECK(esp_board_init(16000, 1, 32));
    afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    afe_config.aec_init = false;
    afe_config.pcm_config.total_ch_num = 2;
    afe_config.pcm_config.mic_num = 1;
    afe_config.pcm_config.ref_num = 1;
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);
    task_flag = 1;

    if(xTaskCreatePinnedToCore(&detect_Task, "detect", 30 * 1024, (void*)afe_data, 12, NULL, 1) != pdPASS){
            ESP_LOGE(TAGMAIN, "Failed to create detect_Task task");
    }
    else{
        ESP_LOGI(TAGMAIN, "Created detect_Task task");
    }
    if(xTaskCreatePinnedToCore(&feed_Task, "feed", 15 * 1024, (void*)afe_data, 12, NULL, 0) != pdPASS){
        ESP_LOGE(TAGMAIN, "Failed to create feed_Task task");
    }
    else{
        ESP_LOGI(TAGMAIN, "Created feed_Task task");
    }
}

void queue_init_main(){
    msg_queue = xQueueCreate(20, sizeof(msg_struct));
    if (msg_queue == NULL) {
        ESP_LOGE(TAGMAIN, "Failed to create queue");
    }
}
void send_msg_to_stm32(void *pvParameters) {
    
    msg_struct msg_data;
    char uart_data[2048];
    while (1) {
        if (xQueueReceive(msg_queue, &msg_data, portMAX_DELAY)) {
            switch (msg_data.type) {
                case DataType_Control:
                    snprintf(uart_data, sizeof(uart_data), "{\"Type\":\"Control\",\"L\":%d,\"R\":%d,\"A\":%d}\n",
                                    msg_data.leftWheelSpeed,
                                    msg_data.rightWheelSpeed,
                                    msg_data.action);

                            // Check wheel speeds to control engine sound playback
                            // if ((msg_data.leftWheelSpeed != 0) || (msg_data.rightWheelSpeed != 0)) {
                            //     start_music_playback();  // Start music playback if any wheel is moving
                            // } else {
                            //     stop_music_playback();  // Stop music playback if both wheels stop
                            // }
                            break;

                case DataType_Weather:
                    snprintf(uart_data, sizeof(uart_data), "{\"Type\":\"Weather\",\"Data\":\"%s\"}\n",
                             msg_data.weatherData);
                    break;

                case DataType_PID:
                    snprintf(uart_data, sizeof(uart_data), "{\"Type\":\"PID\",\"Balance_Kp\":%f,\"Balance_Ki\":%f,\"Balance_Kd\":%f,\"Velocity_Kp\":%f,\"Velocity_Ki\":%f,\"Velocity_Kd\":%f,\"BalanceShift\":%f}\n",
                            msg_data.balancePID.Kp,
                            msg_data.balancePID.Ki,
                            msg_data.balancePID.Kd,
                            msg_data.velocityPID.Kp,
                            msg_data.velocityPID.Ki,
                            msg_data.velocityPID.Kd,
                            msg_data.balanceshift);
                    break;

                case DataType_Mode_Control:
                    snprintf(uart_data, sizeof(uart_data), "{\"Type\":\"Mode_Control\"}\n");
                    break;

                default:
                    ESP_LOGE(TAGMAIN, "Unknown data type received");
                    continue;  // Skip the uart_write_bytes if the type is unknown
            }

            uart_write_bytes(UART_NUM_1, uart_data, strlen(uart_data));
            ESP_LOGI(TAGMAIN, "Sent to STM32: %s", uart_data);
        }
    }
}

// Main application
void app_main(void) {
    nvs_init_main();
    queue_init_main();
    init_uart();
    // setup_log_redirection();

    ble_init_main();
    // wifi_init_main();

    // if(xTaskCreatePinnedToCore(tcp_server_task, "tcp_server_task", 4096, NULL, 11, &tcp_server_task_handle, APP_CPU_NUM) != pdPASS)
    // {
    //     ESP_LOGE(TAGMAIN, "Failed to create tcp_server_task task");
    // }
    // else{
    //     ESP_LOGI(TAGMAIN, "Created tcp_server_task task");
    // }

    initialize_voice_recognition();

    if(xTaskCreatePinnedToCore(send_msg_to_stm32, "send_msg_to_stm32", 8192, NULL, 10, &send_msg_to_stm32_task_handle, APP_CPU_NUM) != pdPASS){
        ESP_LOGE(TAGMAIN, "Failed to create send_msg_to_stm32 task");
    }
    else{
        ESP_LOGI(TAGMAIN, "Created send_msg_to_stm32 task");
    }

    // i2s_speaker_init();
    // if(xTaskCreatePinnedToCore(i2s_speaker_task, "i2s_speaker_task", 8192, NULL, 10, &i2s_speaker_task_handle, APP_CPU_NUM) != pdPASS){
    //     ESP_LOGE(TAGMAIN, "Failed to create i2s_speaker_task task");
    // }
    // else{
    //     ESP_LOGI(TAGMAIN, "Created i2s_speaker_task task");
    // }
    // i2s_mic_init();
    // if(xTaskCreatePinnedToCore(i2s_mic_task, "i2s_mic_task", 8192, NULL, 10, &i2s_mic_task_handle, PRO_CPU_NUM) != pdPASS){
    //     ESP_LOGE(TAGMAIN, "Failed to create i2s_mic_task task");
    // }
    // else{
    //     ESP_LOGI(TAGMAIN, "Created i2s_mic_task task");
    // }

    size_t free_heap = xPortGetFreeHeapSize();
    printf("Free heap size: %d bytes\n", free_heap);
}
