/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f1xx_hal_flash.h"
//Gyro
#include "Gyro.h"
//Motor
#include "Motor.h"
//Encoder
#include "Encoder.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for msg_rx_t */
osThreadId_t msg_rx_tHandle;
const osThreadAttr_t msg_rx_t_attributes = {
  .name = "msg_rx_t",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Motor_update_t */
osThreadId_t Motor_update_tHandle;
const osThreadAttr_t Motor_update_t_attributes = {
  .name = "Motor_update_t",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for msg_rx_q */
osMessageQueueId_t msg_rx_qHandle;
const osMessageQueueAttr_t msg_rx_q_attributes = {
  .name = "msg_rx_q"
};
/* USER CODE BEGIN PV */
//Car_ctr&setup_data
#define FLASH_PID_PARAMS_ADDRESS 0x0800ff00
#define RX_BUFFER_SIZE_1 150
uint8_t rxBuffer_1[RX_BUFFER_SIZE_1];
uint8_t rxIndex_1 = 0;
uint8_t rxData_1;
//#define RX_BUFFER_SIZE_2 64
//uint8_t rxBuffer_2[RX_BUFFER_SIZE_2];
//uint8_t rxIndex_2 = 0;
//uint8_t rxData_2;
typedef enum {
    DataType_Control,
    DataType_Weather,
    DataType_PID,
    DataType_Mode_Control,
    DataType_Unknown // Useful for error handling
} DataType;
typedef struct {
    DataType type;
    uint8_t msg_Data[RX_BUFFER_SIZE_1];
    int L; // Left wheel speed
    int R; // Right wheel speed
    int A; // Additional action or mode

    struct {
        float Balance_Kp, Balance_Ki, Balance_Kd; // Balance PID
        float Velocity_Kp, Velocity_Ki, Velocity_Kd; // Velocity PID
        float Balance_Shift;
    } PID;

    struct {
        char weather[32]; // Description of the weather
        int temperature; // Temperature
        char date[16]; // Date in YYYY-MM-DD format
        char time[16]; // Time HH:MM format
    } WeatherData;
} msg_data_t;
MotorControl motor_R, motor_L;
float Balance_Kp, Balance_Ki, Balance_Kd;
float Velocity_Kp, Velocity_Ki, Velocity_Kd;
float Balance_Shift;
#define DEFAULT_BALANCE_KP 263.0f
#define DEFAULT_BALANCE_KI 0.0f
#define DEFAULT_BALANCE_KD 0.4f
#define DEFAULT_VELOCITY_KP 0.6f
#define DEFAULT_VELOCITY_KI 0.0025f
#define DEFAULT_VELOCITY_KD 0.0f
#define DEFAULT_DEFAULT_BAL_SHIFT -1.5f
uint8_t ctr_cmd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void msg_rx(void *argument);
void Motor_update(void *argument);

/* USER CODE BEGIN PFP */
int _write(int file , char *ptr,int len)
{
    int i = 0;
    for(i = 0;i<len;i++)
        ITM_SendChar((*ptr++));
    return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Timer_Start(void)
{
// ***** CN2 CN4 *****
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);

// ***** CN1 CN3 *****
//	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
//	  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
//	  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
//	  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
//	  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
}
int is_nan(float value) {
    // Check standard isnan
    if (isnan(value)) return 1;

    // Additional check for NaN representation using union
    union {
        float f;
        uint32_t u;
    } converter;

    converter.f = value;
    return (converter.u & 0x7fffffff) > 0x7f800000;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  BSP_MPU6050_Init();
  Timer_Start();
  ReadPIDParamsFromFlash(&Balance_Kp, &Balance_Ki, &Balance_Kd, &Velocity_Kp, &Velocity_Ki, &Velocity_Kd, &Balance_Shift);

// ***** CN2 CN4 *****
  initMotorControl(&motor_R, TIM_CHANNEL_1, TIM_CHANNEL_2, GetEncoder_Counter_R);
  initMotorControl(&motor_L, TIM_CHANNEL_1, TIM_CHANNEL_2, GetEncoder_Counter_L);

// ***** CN1 CN3 *****
//  initMotorControl(&motor_R, TIM_CHANNEL_3, TIM_CHANNEL_4, GetEncoder_Counter_R);
//  initMotorControl(&motor_L, TIM_CHANNEL_3, TIM_CHANNEL_4, GetEncoder_Counter_L);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of msg_rx_q */
  msg_rx_qHandle = osMessageQueueNew (10, sizeof(msg_data_t), &msg_rx_q_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of msg_rx_t */
  msg_rx_tHandle = osThreadNew(msg_rx, NULL, &msg_rx_t_attributes);

  /* creation of Motor_update_t */
  Motor_update_tHandle = osThreadNew(Motor_update, NULL, &Motor_update_t_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 9;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
 __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
HAL_UART_Receive_DMA(&huart2, rxBuffer_1, sizeof(rxBuffer_1));
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Running_Mode_GPIO_Port, Running_Mode_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Charging_Mode_GPIO_Port, Charging_Mode_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Running_Mode_Pin */
  GPIO_InitStruct.Pin = Running_Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Running_Mode_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Mode_Detection_Pin */
  GPIO_InitStruct.Pin = Mode_Detection_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Mode_Detection_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Charging_Mode_Pin */
  GPIO_InitStruct.Pin = Charging_Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Charging_Mode_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Uart Callback

HAL_StatusTypeDef WritePIDParamsToFlash(float Balance_Kp, float Balance_Ki, float Balance_Kd, float Velocity_Kp, float Velocity_Ki, float Velocity_Kd, float Balance_Shift) {
    HAL_StatusTypeDef status;
    uint32_t data[7];
    uint32_t address = FLASH_PID_PARAMS_ADDRESS;

    memcpy(&data[0], &Balance_Kp, sizeof(float));
    memcpy(&data[1], &Balance_Ki, sizeof(float));
    memcpy(&data[2], &Balance_Kd, sizeof(float));
    memcpy(&data[3], &Velocity_Kp, sizeof(float));
    memcpy(&data[4], &Velocity_Ki, sizeof(float));
    memcpy(&data[5], &Velocity_Kd, sizeof(float));
    memcpy(&data[6], &Balance_Shift, sizeof(float));

    //FLASH_Unlock
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t SectorError;
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = FLASH_PID_PARAMS_ADDRESS;
    eraseInitStruct.NbPages = 1;
    status = HAL_FLASHEx_Erase(&eraseInitStruct, &SectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    /* Write the PID parameters to flash, word by word (32 bits) */
    for (uint32_t i = 0; i < (sizeof(data) / sizeof(uint32_t)); i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i]);
        if (status != HAL_OK) {
            break;
        }
        address += 4;
    }
    //FLASH_Lock
    HAL_FLASH_Lock();

    return status;
}

void ReadPIDParamsFromFlash(float *Balance_Kp, float *Balance_Ki, float *Balance_Kd, float *Velocity_Kp, float *Velocity_Ki, float *Velocity_Kd, float *Balance_Shift) {
    memcpy(Balance_Kp, (float*)FLASH_PID_PARAMS_ADDRESS, sizeof(float));
    memcpy(Balance_Ki, (float*)(FLASH_PID_PARAMS_ADDRESS + 4), sizeof(float));
    memcpy(Balance_Kd, (float*)(FLASH_PID_PARAMS_ADDRESS + 8), sizeof(float));
    memcpy(Velocity_Kp, (float*)(FLASH_PID_PARAMS_ADDRESS + 12), sizeof(float));
    memcpy(Velocity_Ki, (float*)(FLASH_PID_PARAMS_ADDRESS + 16), sizeof(float));
    memcpy(Velocity_Kd, (float*)(FLASH_PID_PARAMS_ADDRESS + 20), sizeof(float));
    memcpy(Balance_Shift, (float*)(FLASH_PID_PARAMS_ADDRESS + 24), sizeof(float));

    // Check for NaN values and set to default if necessary
    if (is_nan(*Balance_Kp)) *Balance_Kp = DEFAULT_BALANCE_KP;
    if (is_nan(*Balance_Ki)) *Balance_Ki = DEFAULT_BALANCE_KI;
    if (is_nan(*Balance_Kd)) *Balance_Kd = DEFAULT_BALANCE_KD;
    if (is_nan(*Velocity_Kp)) *Velocity_Kp = DEFAULT_VELOCITY_KP;
    if (is_nan(*Velocity_Ki)) *Velocity_Ki = DEFAULT_VELOCITY_KI;
    if (is_nan(*Velocity_Kd)) *Velocity_Kd = DEFAULT_VELOCITY_KD;
    if (is_nan(*Balance_Shift)) *Balance_Shift = DEFAULT_DEFAULT_BAL_SHIFT;
}

void UART_IRQHandler_IDLE(UART_HandleTypeDef *huart)
{
	msg_data_t ctr;
	if(huart == &huart2)
	{
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
		{
			HAL_UART_DMAStop(&huart2);
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);
			__HAL_UART_CLEAR_OREFLAG(&huart2);
			memcpy(ctr.msg_Data ,rxBuffer_1, sizeof(rxBuffer_1));
			osStatus_t status = osMessageQueuePut(msg_rx_qHandle, &ctr, 0, 0);
			if (status != osOK) {printf("Failed to put message into queue\n");}
			HAL_UART_Receive_DMA(&huart2, rxBuffer_1, sizeof(rxBuffer_1));
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_msg_rx */
/**
  * @brief  Function implementing the msg_rx_t thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_msg_rx */
void msg_rx(void *argument)
{
  /* USER CODE BEGIN 5 */
	msg_data_t ctr;
	char type[32];
	for(;;) {
		if (osMessageQueueGet(msg_rx_qHandle, &ctr, NULL, osWaitForever) == osOK) {
			printf("%s\n",ctr.msg_Data);
			if (sscanf((char*)ctr.msg_Data, "{\"Type\":\"%[^\"]\"", type) == 1) {
				if (strcmp(type, "Control") == 0) {
					if (sscanf((char*)ctr.msg_Data, "{\"Type\":\"Control\",\"L\":%d,\"R\":%d,\"A\":%d}",
							   &ctr.L, &ctr.R, &ctr.A) == 3) {
						motor_R.targetSpeed = ctr.R;
						motor_L.targetSpeed = ctr.L;
						ctr_cmd = ctr.A;
						ctr.type = DataType_Control;
					}
				}
				else if (strcmp(type, "Weather") == 0) {
					if (sscanf((char*)ctr.msg_Data, "{\"Type\":\"Weather\",\"Weather\":\"%[^\"]\",\"Temperature\":%d,\"Date\":\"%[^\"]\",\"Time\":\"%[^\"]\"}",
							   ctr.WeatherData.weather,
							   &ctr.WeatherData.temperature,
							   ctr.WeatherData.date,
							   ctr.WeatherData.time) == 4) {
						printf("Weather: %s, Temp: %d, Date: %s, Time: %s\n",
							   ctr.WeatherData.weather,
							   ctr.WeatherData.temperature,
							   ctr.WeatherData.date,
							   ctr.WeatherData.time);
						ctr.type = DataType_Weather;
					}
				} else if (strcmp(type, "PID") == 0) {
					if (sscanf((char*)ctr.msg_Data, "{\"Type\":\"PID\",\"Balance_Kp\":%f,\"Balance_Ki\":%f,\"Balance_Kd\":%f,"
							"\"Velocity_Kp\":%f,\"Velocity_Ki\":%f,\"Velocity_Kd\":%f,\"Balance_Shift\":%f}",
							   &ctr.PID.Balance_Kp,
							   &ctr.PID.Balance_Ki,
							   &ctr.PID.Balance_Kd,
							   &ctr.PID.Velocity_Kp,
							   &ctr.PID.Velocity_Ki,
							   &ctr.PID.Velocity_Kd,
							   &ctr.PID.Balance_Shift) == 7) {
						Balance_Kp = ctr.PID.Balance_Kp;
						Balance_Ki = ctr.PID.Balance_Ki;
						Balance_Kd = ctr.PID.Balance_Kd;
						Velocity_Kp = ctr.PID.Velocity_Kp;
						Velocity_Ki = ctr.PID.Velocity_Ki;
						Velocity_Kd = ctr.PID.Velocity_Kd;
						Balance_Shift = ctr.PID.Balance_Shift;
						ctr.type = DataType_PID;
						if (WritePIDParamsToFlash(Balance_Kp, Balance_Ki, Balance_Kd, Velocity_Kp, Velocity_Ki, Velocity_Kd, Balance_Shift) != HAL_OK) {
							printf("PID_store_to_flash_failed\n");
						}
					}
				}
				else if (strcmp(type, "Mode_Control") == 0) {
					printf("Mode control switch\n");
					ctr.type = DataType_Mode_Control;
				} else {
					printf("Unknown message type\n");
					ctr.type = DataType_Unknown;
				}
			}
		}
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Motor_update */
/**
* @brief Function implementing the Motor_update_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_update */
void Motor_update(void *argument)
{
  /* USER CODE BEGIN Motor_update */
	  int count = 1;

	  /* Infinite loop */
	  for(;;)
	  {
		  read_angle(&motor_R, &motor_L);
		  if (count % 10 == 0) {
			Velocity_PID_Control(&motor_R, &motor_L);
		  }

		  Balance_PID_Control(&motor_R, &motor_L);

		  if (HAL_GPIO_ReadPin(GPIOC, Mode_Detection_Pin) == GPIO_PIN_RESET) {
			UpdateMotorPWM(&motor_R, &motor_L);
		  }

		  osDelay(5);
		  count = (count ) % 10 == 0 ? 1: count+1;
	  }
  /* USER CODE END Motor_update */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
