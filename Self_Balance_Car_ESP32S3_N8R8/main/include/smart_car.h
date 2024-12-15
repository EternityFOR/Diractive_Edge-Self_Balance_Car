#ifndef MAIN_H
#define MAIN_H

typedef struct {
    bool touch0_triggered;
    bool touch1_triggered;
    bool hs0_triggered;
    bool hs1_triggered;   
    bool hs2_triggered;   
    bool lidar_triggered; 
    bool rfid_triggered;  
    bool ads_triggered;   
} sensor_status_t;

typedef struct {
    int32_t button_debounce;
    int32_t command_resent_times;
    int32_t motor_pwm;
    int32_t rgb_pwm;
    int32_t motor_restart_delay;
    int32_t hall_sensor_debounce;
    int32_t home_found_dump_duration;
    int32_t home_found_reset_duration;
    int32_t home_found_delay;
    int32_t h_r_angle_stop_delay;
} adjustable_params_t;
void send_msg_to_stm32(void *pvParameters);
void save_params_to_nvs();
#endif
