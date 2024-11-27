// Data_handle.c
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include <string.h>
#include "Data_handle.h"
#include "blufi_app.h"
// Tags and Logging Identifier
#define GATTS_DATA_TAG "GATTS_DATA_TAG"

// External Variables
extern QueueHandle_t msg_queue;
TimerHandle_t switch_back_to_original_ble_timer = NULL;
// Functions
void handle_pid_data(cJSON *root) {
    cJSON *balancePID = cJSON_GetObjectItemCaseSensitive(root, "balancePID");
    cJSON *velocityPID = cJSON_GetObjectItemCaseSensitive(root, "velocityPID");
    cJSON *balanceshift = cJSON_GetObjectItemCaseSensitive(root, "balanceshift");

    if (!balancePID || !velocityPID) {
        ESP_LOGE(GATTS_DATA_TAG, "PID JSON does not contain required fields");
        return;
    }

    msg_struct pid_data;
    pid_data.type = DataType_PID;

    pid_data.balancePID.Kp = cJSON_GetObjectItemCaseSensitive(balancePID, "Kp")->valuedouble;
    pid_data.balancePID.Ki = cJSON_GetObjectItemCaseSensitive(balancePID, "Ki")->valuedouble;
    pid_data.balancePID.Kd = cJSON_GetObjectItemCaseSensitive(balancePID, "Kd")->valuedouble;

    pid_data.velocityPID.Kp = cJSON_GetObjectItemCaseSensitive(velocityPID, "Kp")->valuedouble;
    pid_data.velocityPID.Ki = cJSON_GetObjectItemCaseSensitive(velocityPID, "Ki")->valuedouble;
    pid_data.velocityPID.Kd = cJSON_GetObjectItemCaseSensitive(velocityPID, "Kd")->valuedouble;
        // Add check for balanceshift
    if (balanceshift != NULL) {
        pid_data.balanceshift = balanceshift->valuedouble;
    } else {
        pid_data.balanceshift = 0.0;  // Set a default value if balanceshift is not present
    }

    if (xQueueSend(msg_queue, &pid_data, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(GATTS_DATA_TAG, "Failed to send PID data to queue");
    }
}

void handle_car_control_signal(char *car_control_data_str) {
    cJSON *root = cJSON_Parse(car_control_data_str);
    if (root == NULL) {
        ESP_LOGE(GATTS_DATA_TAG, "Error parsing Car Control data");
        return;
    }

    cJSON *leftWheelSpeedItem = cJSON_GetObjectItemCaseSensitive(root, "leftWheelSpeed");
    cJSON *rightWheelSpeedItem = cJSON_GetObjectItemCaseSensitive(root, "rightWheelSpeed");

    if (!leftWheelSpeedItem || !rightWheelSpeedItem) {
        ESP_LOGE(GATTS_DATA_TAG, "Car Control JSON does not contain required fields");
        cJSON_Delete(root);
        return;
    }

    msg_struct car_control_data = {
        .type = DataType_Control,
        .leftWheelSpeed = leftWheelSpeedItem->valueint,
        .rightWheelSpeed = rightWheelSpeedItem->valueint,
    };

    if (xQueueSend(msg_queue, &car_control_data, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(GATTS_DATA_TAG, "Failed to send car control data to queue");
    }

    cJSON_Delete(root);
}

void handle_car_control_mode() {

    msg_struct mode_control_data = {.type = DataType_Mode_Control};

    if (xQueueSend(msg_queue, &mode_control_data, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(GATTS_DATA_TAG, "Failed to send mode control signal to queue");
    }
}

void handle_connect_mode() {
    // Implement the connection mode logic
}

void handle_blufi_mode() {
    blufi_init();
    // Initialize BLUFI and set up a timeout timer to switch back to the original BLE mode
    switch_back_to_original_ble_timer = xTimerCreate("SwitchBLETmr", pdMS_TO_TICKS(10000), pdFALSE, (void*)0, switch_back_to_original_ble_callback);
    if (switch_back_to_original_ble_timer == NULL) {
        ESP_LOGE(GATTS_DATA_TAG, "Timeout timer creation failed");
    }
    if (xTimerStart(switch_back_to_original_ble_timer, 0) != pdPASS) {
        ESP_LOGE(GATTS_DATA_TAG, "Timeout timer start failed");
    }
}

void switch_back_to_original_ble_callback(TimerHandle_t xTimer) {
    esp_restart();
}