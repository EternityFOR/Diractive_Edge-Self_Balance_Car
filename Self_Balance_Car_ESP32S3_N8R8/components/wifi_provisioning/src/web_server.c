#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "smart_car.h"

extern sensor_status_t sensor_status;
extern adjustable_params_t params;

#define TAGWEBSERVER "web_server"

static esp_err_t sensor_states_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();

    cJSON_AddBoolToObject(root, "touch0_triggered", sensor_status.touch0_triggered);
    cJSON_AddBoolToObject(root, "touch1_triggered", sensor_status.touch1_triggered);
    cJSON_AddBoolToObject(root, "hs0_triggered", sensor_status.hs0_triggered);
    cJSON_AddBoolToObject(root, "hs1_triggered", sensor_status.hs1_triggered);
    cJSON_AddBoolToObject(root, "hs2_triggered", sensor_status.hs2_triggered);
    cJSON_AddBoolToObject(root, "lidar_triggered", sensor_status.lidar_triggered);
    cJSON_AddBoolToObject(root, "rfid_triggered", sensor_status.rfid_triggered);
    cJSON_AddBoolToObject(root, "ads_triggered", sensor_status.ads_triggered);

    const char *json_response = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_response);

    cJSON_Delete(root);
    free((void *)json_response);
    return ESP_OK;
}

static esp_err_t get_params_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "button_debounce", params.button_debounce);
    cJSON_AddNumberToObject(root, "command_resent_times", params.command_resent_times);
    cJSON_AddNumberToObject(root, "motor_pwm", params.motor_pwm);
    cJSON_AddNumberToObject(root, "rgb_pwm", params.rgb_pwm);
    cJSON_AddNumberToObject(root, "motor_restart_delay", params.motor_restart_delay);
    cJSON_AddNumberToObject(root, "hall_sensor_debounce", params.hall_sensor_debounce);
    cJSON_AddNumberToObject(root, "home_found_dump_duration", params.home_found_dump_duration);
    cJSON_AddNumberToObject(root, "home_found_reset_duration", params.home_found_reset_duration);
    cJSON_AddNumberToObject(root, "home_found_delay", params.home_found_delay);
    cJSON_AddNumberToObject(root, "h_r_angle_stop_delay", params.h_r_angle_stop_delay);

    const char *json_response = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_response);

    cJSON_Delete(root);
    free((void *)json_response);
    return ESP_OK;
}

static esp_err_t update_params_handler(httpd_req_t *req) {
    char buf[512];
    int ret, remaining = req->content_len;

    bzero(buf, sizeof(buf));
    while (remaining > 0) {
        // ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        ret = httpd_req_recv(req, buf, remaining);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        remaining -= ret;
    }

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_sendstr(req, "Invalid JSON data");
        return ESP_FAIL;
    }

    params.button_debounce = cJSON_GetObjectItem(root, "button_debounce")->valueint;
    params.command_resent_times = cJSON_GetObjectItem(root, "command_resent_times")->valueint;
    params.motor_pwm = cJSON_GetObjectItem(root, "motor_pwm")->valueint;
    params.rgb_pwm = cJSON_GetObjectItem(root, "rgb_pwm")->valueint;
    params.motor_restart_delay = cJSON_GetObjectItem(root, "motor_restart_delay")->valueint;
    params.hall_sensor_debounce = cJSON_GetObjectItem(root, "hall_sensor_debounce")->valueint;
    params.home_found_dump_duration = cJSON_GetObjectItem(root, "home_found_dump_duration")->valueint;
    params.home_found_reset_duration = cJSON_GetObjectItem(root, "home_found_reset_duration")->valueint;
    params.home_found_delay = cJSON_GetObjectItem(root, "home_found_delay")->valueint;
    params.h_r_angle_stop_delay = cJSON_GetObjectItem(root, "h_r_angle_stop_delay")->valueint;

    cJSON_Delete(root);
    save_params_to_nvs(params);
    httpd_resp_sendstr(req, "Parameters updated successfully");
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    extern const unsigned char upload_script_start[] asm("_binary_sensors_html_start");
    extern const unsigned char upload_script_end[] asm("_binary_sensors_html_end");
    const size_t upload_script_size = (upload_script_end - upload_script_start);
    httpd_resp_send(req, (const char *)upload_script_start, upload_script_size);
    return ESP_OK;
}

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    // Set status
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAGWEBSERVER, "Redirecting to root");
    return ESP_OK;
}

esp_err_t start_web_server() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;

    ESP_LOGI(TAGWEBSERVER, "Starting HTTP Server");
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAGWEBSERVER, "Failed to start server!");
        return ESP_FAIL;
    }

    httpd_uri_t root = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = root_get_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &root);
    
    httpd_uri_t sensor_states = {
        .uri       = "/sensor_states",
        .method    = HTTP_GET,
        .handler   = sensor_states_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &sensor_states);

    httpd_uri_t get_params = {
        .uri       = "/get_params",
        .method    = HTTP_GET,
        .handler   = get_params_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_params);


    httpd_uri_t update_params = {
        .uri       = "/update_params",
        .method    = HTTP_POST,
        .handler   = update_params_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &update_params);
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
    return ESP_OK;
}
