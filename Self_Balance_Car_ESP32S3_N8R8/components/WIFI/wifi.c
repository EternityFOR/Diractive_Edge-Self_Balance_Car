#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "Data_handle.h"

#define MAX_HTTP_OUTPUT_BUFFER 4096*5
static const char *TAG = "Weather_Task";

static char global_json_data[MAX_HTTP_OUTPUT_BUFFER] = {0};

static const char *WEATHER_URL = "http://api.seniverse.com/v3/weather/now.json?key=S11nWrwtdvX3CmHUB&location=xianggang&language=en&unit=c";

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT_t = BIT0;

extern QueueHandle_t msg_queue;

static void wait_for_wifi_connection() {
    wifi_ap_record_t ap_info;
    esp_err_t wifi_status = esp_wifi_sta_get_ap_info(&ap_info);
    while (wifi_status != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        wifi_status = esp_wifi_sta_get_ap_info(&ap_info);
    }
    xEventGroupSetBits(wifi_event_group, CONNECTED_BIT_t);
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA: 
         if (evt->data_len < MAX_HTTP_OUTPUT_BUFFER) {
            memcpy(evt->user_data + strlen(evt->user_data), evt->data, evt->data_len);
            ((char*)evt->user_data)[evt->data_len] = '\0';
        }
        ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, data =%s, len=%d", evt->user_data, strlen(evt->user_data));
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
    default:
        ESP_LOGD(TAG, "Unhandled HTTP Event: %d", evt->event_id);
        break;
    }
    return ESP_OK;
}

static void http_weather_task(void *pvParameters) {
    wifi_event_group = xEventGroupCreate();
    
    while (1) {
        wait_for_wifi_connection();
        
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT_t, false, true, portMAX_DELAY);

        ESP_LOGI(TAG, "Wi-Fi connected, starting HTTP request.");

        int max_retry = 10;
        int retry_count = 0;
        esp_err_t err = ESP_FAIL;
        bool request_successful = false;

        while (retry_count < max_retry && !request_successful) {
            char output_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

            esp_http_client_config_t config = {
                .url = WEATHER_URL,
                .event_handler = http_event_handler,
                .user_data = output_buffer,
                .disable_auto_redirect = false,
                .skip_cert_common_name_check = true,
                .cert_pem = NULL,
                .buffer_size = MAX_HTTP_OUTPUT_BUFFER,
            };

            esp_http_client_handle_t client = esp_http_client_init(&config);

            err = esp_http_client_perform(client);

        if (err == ESP_OK) {
        cJSON *root = cJSON_Parse(output_buffer);
                if (root != NULL) 
                {
                    cJSON *results = cJSON_GetObjectItem(root, "results");
                    if (results != NULL) {
                        cJSON *first_result = cJSON_GetArrayItem(results, 0);
                        if (first_result != NULL) {
                            // cJSON *location = cJSON_GetObjectItem(first_result, "location");
                            cJSON *now = cJSON_GetObjectItem(first_result, "now");
                            cJSON *last_update = cJSON_GetObjectItem(first_result, "last_update");

                            cJSON *simplified_root = cJSON_CreateObject();
                            // if (location) cJSON_AddItemToObject(simplified_root, "location", cJSON_Duplicate(location, true));
                            if (now) {
                                cJSON *weather = cJSON_GetObjectItem(now, "text");
                                cJSON *temperature = cJSON_GetObjectItem(now, "temperature");
                                if (weather) cJSON_AddItemToObject(simplified_root, "weather", cJSON_Duplicate(weather, true));
                                if (temperature) cJSON_AddItemToObject(simplified_root, "temperature", cJSON_Duplicate(temperature, true));
                            }
                            if (last_update) {
                                char *date_time = cJSON_GetStringValue(last_update);
                                if (date_time) {
                                    //"YYYY-MM-DDTHH:MM:SS+HH:MM"
                                    char date[11]; // "YYYY-MM-DD\0"
                                    char time[9];  // "HH:MM:SS\0"
                                    sscanf(date_time, "%10sT%8s", date, time);
                                    cJSON_AddStringToObject(simplified_root, "date", date);
                                    cJSON_AddStringToObject(simplified_root, "time", time);
                                }
                            }

                            char *rendered = cJSON_Print(simplified_root);
                            strncpy(global_json_data, rendered, MAX_HTTP_OUTPUT_BUFFER);

                            //Send data to queue
                            msg_struct weather_time_Data = {
                                .type = DataType_Weather,
                                .leftWheelSpeed = 0,
                                .rightWheelSpeed = 0,
                                .action = 0,
                                .weatherData = ""
                            };
                            strcpy(weather_time_Data.weatherData, global_json_data);
                            ESP_LOGI(TAG, "Global JSON data: %s", global_json_data);
                            if (xQueueSend(msg_queue, &weather_time_Data, portMAX_DELAY) != pdPASS) {
                                ESP_LOGE(TAG, "Failed to send weather_time data to queue");
                            }
                            free(rendered);
                            cJSON_Delete(simplified_root);
                        }
                    }
                    cJSON_Delete(root);

                } 
                else 
                {
                    ESP_LOGE(TAG, "Json parse: %s", esp_err_to_name(err));
                }
            request_successful = true;
        }
        else
        {
            ESP_LOGE(TAG, "HTTP GET request failed: %s, retrying...", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        }

        if (!request_successful) {
            ESP_LOGE(TAG, "Failed to get HTTP response after %d retries.", max_retry);
        }
        vTaskDelay(pdMS_TO_TICKS(3600000));
    }
}


void weather_time_task_init(void) {
    xTaskCreate(&http_weather_task, "http_weather_task", 4096*10, NULL, 2, NULL);
}