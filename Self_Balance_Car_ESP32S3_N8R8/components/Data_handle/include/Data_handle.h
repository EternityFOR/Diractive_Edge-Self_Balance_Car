// Data_handle.h
#ifndef DATA_HANDLE_H
#define DATA_HANDLE_H
#include "freertos/timers.h"
#include "cJSON.h"

typedef enum {
    DataType_Control,
    DataType_Weather,
    DataType_PID,
    DataType_Mode_Control,
    DataType_Mode_Connect,
    DataType_BLUFI
} DataType;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PIDParams;

typedef struct {
    DataType type;
    int leftWheelSpeed;
    int rightWheelSpeed;
    int action;
    char weatherData[256];
    PIDParams balancePID;
    PIDParams velocityPID;
    float balanceshift;
} msg_struct;

void handle_car_control_signal(char *car_control_data_str);
void handle_pid_data(cJSON *root);
void handle_car_control_mode();
void handle_connect_mode();
void handle_blufi_mode();
void switch_back_to_original_ble_callback(TimerHandle_t xTimer);

#endif