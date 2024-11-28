#include <stdint.h>
#include <stdio.h>
#include "Motor.h"
#include "Encoder.h"
#include "Gyro.h"
#include "main.h"
#include "math.h"
#include "string.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))
#define ENCODER_PULSES_PER_REVOLUTION 720
#define TIME_INTERVAL 0.05
#define INTEGRAL_LIMIT 1500.0
#define MAX_PWM 7000
#define MAX_BALANCE_PWM 4200
#define MAX_VELOCITY_PWM 2800
#define SPEED_MAX_THRESHOLD 4000
#define SPEED_DIFF_THRESHOLD 500
#define MIN_MOVE_PWM 800

extern MotorControl motor_R, motor_L;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern float Velocity_Kp, Velocity_Ki, Velocity_Kd;
extern float Balance_Kp, Balance_Ki, Balance_Kd;
extern float current_angle;
extern IMU_SensorData_Raw_Structer IMU_SensorData_Raw;

float Turn_Kp = 1.2f;
float Turn_Kd = 0.8f;
float balance_error = 0.0, balance_integral = 0.0, balance_derivative = 0.0, prev_balance_error = 0.0;
float safety_angle = 50.0;
float sign;
float targetSpeed;
int carLifted = 0;
unsigned flag = 1;
float turn_error = 0.0, prev_turn_error = 0.0, turn_output = 0.0;
<<<<<<< HEAD
extern float BalanceShift;
=======
extern float Balance_Shift;
>>>>>>> 767b406b14e05d6fadf094114ea391546080ae84

void Velocity_PID_Control(MotorControl *motor_R, MotorControl *motor_L) {
    MotorControl* motors[] = {motor_R, motor_L};
    for (int i = 0; i < 2; i++) {
        MotorControl *motor = motors[i];
        CalculateMotorSpeed(motor);
        targetSpeed = motor->targetSpeed;

        if (targetSpeed != 0) {
            sign = targetSpeed > 0 ? 1.0 : -1.0;
            motor->targetAngle = sign * 3.5;
            targetSpeed = sign * 200;
        } else {
            motor->targetAngle = 0.0;
        }

        float velocityError = targetSpeed - motor->currentSpeed;
        motor->velocityError = motor->velocityError * 0.8f + velocityError * 0.2f;

        if ((motor->velocityPrevError * velocityError) < 0) {
            motor->velocityIntegral *= 0.3;
        }

        motor->velocityIntegral += motor->velocityError * TIME_INTERVAL;
        if (motor->velocityIntegral > INTEGRAL_LIMIT) {
            motor->velocityIntegral = INTEGRAL_LIMIT;
        } else if (motor->velocityIntegral < -INTEGRAL_LIMIT) {
            motor->velocityIntegral = -INTEGRAL_LIMIT;
        }

        motor->velocityOutput = Velocity_Kp * motor->velocityError + Velocity_Ki * motor->velocityIntegral;
        motor->velocityOutput = fminf(fmaxf(motor->velocityOutput, -MAX_VELOCITY_PWM), MAX_VELOCITY_PWM);
        motor->velocityPrevError = motor->velocityError;
    }
}

void Balance_PID_Control(MotorControl *motor_R, MotorControl *motor_L) {
    MotorControl* motors[] = {motor_R, motor_L};
    for (int i = 0; i < 2; i++) {
        MotorControl *motor = motors[i];
<<<<<<< HEAD
        balance_error = motor->currentAngle - motor->targetAngle - BalanceShift;
=======
        balance_error = motor->currentAngle - motor->targetAngle - Balance_Shift;
>>>>>>> 767b406b14e05d6fadf094114ea391546080ae84
        motor->balanceOutput = Balance_Kp * balance_error + Balance_Kd * (balance_error - prev_balance_error);
        motor->balanceOutput = fminf(fmaxf(motor->balanceOutput, -MAX_BALANCE_PWM), MAX_BALANCE_PWM);
        prev_balance_error = balance_error;
    }
}

void Turn_PD_Control(MotorControl *motor_R, MotorControl *motor_L) {
    float speed_difference = -(motor_L->currentSpeed - motor_R->currentSpeed);
    turn_error = speed_difference;

    turn_output = Turn_Kp * turn_error + Turn_Kd * (turn_error - prev_turn_error);
    prev_turn_error = turn_error;
    if ((motor_R->targetSpeed != 0 && motor_L->targetSpeed == 0) || (motor_R->targetSpeed == 0 && motor_L->targetSpeed != 0)) {
    	turn_output = 0.0f;
    }
}

void UpdateMotorPWM(MotorControl *motor_R, MotorControl *motor_L) {
    if (flag && (isCarLifted(motor_R, motor_L) || fabs(motor_R->currentAngle) >= safety_angle || fabs(motor_L->currentAngle) >= safety_angle)) {
        StopMotors(motor_R, motor_L);
        motor_R->velocityIntegral = 0.0f;
        motor_L->velocityIntegral = 0.0f;
        return;
    }

    Turn_PD_Control(motor_R, motor_L);
    motor_R->finalOutput = motor_R->balanceOutput - motor_R->velocityOutput - turn_output;
    motor_L->finalOutput = motor_L->balanceOutput - motor_L->velocityOutput + turn_output;

//    if(((motor_R-> targetSpeed != 0) || (motor_R-> targetSpeed != 0)) && ((motor_R-> currentSpeed < 500) || (motor_L-> currentSpeed < 500))) {
//    	motor_R-> finalOutput = motor_R-> targetSpeed > 0 ? MIN_MOVE_PWM : -MIN_MOVE_PWM;
//    	motor_L-> finalOutput = motor_L-> targetSpeed > 0 ? MIN_MOVE_PWM : -MIN_MOVE_PWM;
//    }
    motor_R-> finalOutput = fminf(fmaxf(motor_R-> finalOutput, -MAX_PWM), MAX_PWM);
    motor_L-> finalOutput = fminf(fmaxf(motor_L-> finalOutput, -MAX_PWM), MAX_PWM);

    if (motor_R->finalOutput >= 0.0f) {
        __HAL_TIM_SET_COMPARE(&htim8, motor_R->negativeChannel, (uint32_t)motor_R->finalOutput);
        __HAL_TIM_SET_COMPARE(&htim8, motor_R->positiveChannel, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim8, motor_R->negativeChannel, 0);
        __HAL_TIM_SET_COMPARE(&htim8, motor_R->positiveChannel, (uint32_t)(-motor_R->finalOutput));
    }

    if (motor_L->finalOutput >= 0.0f) {
        __HAL_TIM_SET_COMPARE(&htim1, motor_L->negativeChannel, (uint32_t)motor_L->finalOutput);
        __HAL_TIM_SET_COMPARE(&htim1, motor_L->positiveChannel, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, motor_L->negativeChannel, 0);
        __HAL_TIM_SET_COMPARE(&htim1, motor_L->positiveChannel, (uint32_t)(-motor_L->finalOutput));
    }
}

void CalculateMotorSpeed(MotorControl *motor) {
    motor->encoder_counter = motor->GetEncoderCounter();
    float encoder_count = motor->encoder_counter / (float)ENCODER_PULSES_PER_REVOLUTION / TIME_INTERVAL * 60.0f;
    motor->currentSpeed = -RPMToPWM((motor == &motor_R) ? -encoder_count : encoder_count);
}

void StopMotors(MotorControl *motor_R, MotorControl *motor_L) {
    __HAL_TIM_SET_COMPARE(&htim8, motor_R->positiveChannel, 0);
    __HAL_TIM_SET_COMPARE(&htim8, motor_R->negativeChannel, 0);
    __HAL_TIM_SET_COMPARE(&htim1, motor_L->positiveChannel, 0);
    __HAL_TIM_SET_COMPARE(&htim1, motor_L->negativeChannel, 0);
}

int isCarLifted(MotorControl *motor_R, MotorControl *motor_L) {
    float speedDifference_R = fabs(fabs(motor_R->currentSpeed) - fabs(motor_R->finalOutput));
    float speedDifference_L = fabs(fabs(motor_L->currentSpeed) - fabs(motor_L->finalOutput));

    if ((fabs(motor_R->currentSpeed) > SPEED_MAX_THRESHOLD || fabs(motor_L->currentSpeed) > SPEED_MAX_THRESHOLD) &&
        (speedDifference_R < SPEED_DIFF_THRESHOLD || speedDifference_L < SPEED_DIFF_THRESHOLD)) {
        carLifted = 1;
    }

    return carLifted;
}

void read_angle(MotorControl *motor_R, MotorControl *motor_L) {
    BSP_MPU6050_UpdateSensors();
    float raw_angle = atan2(IMU_SensorData_Raw.ACC_X, IMU_SensorData_Raw.ACC_Z) * (180.0f / M_PI);
    motor_R->currentAngle = raw_angle;
    motor_L->currentAngle = raw_angle;
}

float RPMToPWM(float targetRPM) {
    return targetRPM / 500 * MAX_PWM;
}

void initMotorControl(MotorControl *motor,
                      uint32_t positiveChannel,
                      uint32_t negativeChannel,
                      int (*GetEncoderCounter)(void)) {
    memset(motor, 0, sizeof(MotorControl));

    motor->positiveChannel = positiveChannel;
    motor->negativeChannel = negativeChannel;
    motor->GetEncoderCounter = GetEncoderCounter;
}
