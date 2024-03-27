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
#include "oled.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/// Movement controls ///
#define RPM_LEFT 3500
#define RPM_RIGHT 3500

// Voltage for IR
#define ADC_REF 3.3

// Resolution for IR
#define ADC_STEPS 4096
volatile uint8_t motionCommand = 0;
volatile uint8_t dirCommand = 0;
volatile uint8_t distCommand = 0;
volatile uint8_t commandReceivedFlag = 0;
typedef struct{
	I2C_HandleTypeDef *i2cHandle;
	UART_HandleTypeDef *uart;
	int16_t gyro;
	float gyroYaw;
}IMU_Data;
IMU_Data imu;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
I2C_HandleTypeDef hi2c1;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GyroTask */
osThreadId_t GyroTaskHandle;
const osThreadAttr_t GyroTask_attributes = {
  .name = "GyroTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
// Comms
uint8_t uartRxBuffer[5]; // UART receive buffer
uint8_t oledBuffer[20]; // Gyro receive buffer

osMessageQueueId_t motorCommandQueueHandle;
const osMessageQueueAttr_t motorCommandQueue_attributes = {
  .name = "motorCommandQueue"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void motor(void *argument);
void encoder(void *argument);
void Gyro(void *argument);

/* USER CODE BEGIN PFP */
float adjustServoToCenter(float, float);

// For IMU and gyro
HAL_StatusTypeDef ret;
HAL_StatusTypeDef IMU_WriteOneByte(IMU_Data *dev, uint8_t reg, uint8_t data);
HAL_StatusTypeDef IMU_ReadOneByte(IMU_Data *dev, uint8_t reg, uint8_t *data);
uint8_t * IMU_Initialise(IMU_Data *dev, I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef *uart);
float yawAngle = 0;
volatile uint8_t dma_transfer_complete = 0;
uint8_t rawData[2];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// For Ultrasonic
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t echo = 0;	// For storing the echo pulse duration
uint8_t Is_First_Captured = 0;  // is the first value captured ? A flag to ensure that both cases are accounted for
uint32_t Distance  = 0;	// For storing the distance

// For Motor
int initialPwmValue = 0;
int encoderL = 0, encoderR = 0;
int pwmValL = 0;
int pwmValR = 0;

// Directions
uint16_t pwmVal_servo = 139;

// Gyro
uint8_t ICMAddress = 0x68;

// imu registers
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38
#define REG_BANK_SEL 0x7F
#define IMU_ADDR	(0x68 << 1)
#define GYRO_CONFIG_1	(0x01)
#define GYRO_CONFIG_2  (0x02)
#define ACCEL_CONFIG	(0x14)
#define ACCEL_CONFIG_2  (0x15)
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2	0x07
#define LP_CONFIG	0x05
#define REG_WHO_AM_I 0x00
#define USER_CTRL  0x03
#define GYRO_SMPLRT_DIV 0x00
static uint32_t time_elapsed = 0x00;
static uint32_t prev_time_elapsed = 0x00;
static uint32_t time_difference = 0x00;
int16_t Data;
float targetAngle = 90.0;

// IR Sensor
volatile uint16_t adcResult[20];
volatile uint16_t ir_left;
volatile uint16_t ir_right;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint8_t hello[20] = "Hello World!\0";
//	int dist = 0;	// For storing the distance

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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motor, NULL, &MotorTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder, NULL, &EncoderTask_attributes);

  /* creation of GyroTask */
  GyroTaskHandle = osThreadNew(Gyro, NULL, &GyroTask_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 100000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  htim8.Init.Prescaler = 0;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TriggerB4_GPIO_Port, TriggerB4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Pin */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TriggerB4_Pin */
  GPIO_InitStruct.Pin = TriggerB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TriggerB4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// Callback Function where the interrupt will go to
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == USART3) {
	        motionCommand = uartRxBuffer[0]; // Store the motion (forward/backward/stop; set rotation mode)
	        dirCommand = uartRxBuffer[1]; // Store the direction (left/center/right)
	        distCommand = ((int) (uartRxBuffer[2]) - 48) * 100
					+ ((int) (uartRxBuffer[3]) - 48) * 10
					+ ((int) (uartRxBuffer[4]) - 48);// Store the distance (cm) / angle (degree) (format: 000)
	        commandReceivedFlag = 1;           // Set the flag to indicate a new command has been received

	        // Re-enable UART receive interrupt for the next command
	        HAL_UART_Receive_IT(&huart3, uartRxBuffer, 5);
	    }

}


// Gyro
uint8_t * IMU_Initialise(IMU_Data *dev, I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef *uart){
	char hex[2];
	uint8_t uartbuf[20] = "  IMU ID =    "; //buffer for data
	uint8_t regData;
	dev->i2cHandle = i2cHandle;
	dev->uart = uart;
	ret = IMU_ReadOneByte(dev, REG_WHO_AM_I, &regData);
	ret = IMU_WriteOneByte(dev, REG_BANK_SEL, 0x00);
	ret = IMU_WriteOneByte(dev, USER_CTRL, 0x20);
	ret = IMU_WriteOneByte(dev, PWR_MGMT_1, 0x09);
	ret = IMU_WriteOneByte(dev, PWR_MGMT_2, 0x3E);
	ret = IMU_WriteOneByte(dev, LP_CONFIG, 0x40);
	ret = IMU_WriteOneByte(dev, USER_CTRL, 0x00);
	ret = IMU_WriteOneByte(dev, REG_BANK_SEL, 0x20);
	ret = IMU_WriteOneByte(dev, GYRO_CONFIG_1, 0x39);
	ret = IMU_WriteOneByte(dev, GYRO_CONFIG_2, 0x00);
	ret = IMU_WriteOneByte(dev, ACCEL_CONFIG, 0x39);
	ret = IMU_WriteOneByte(dev, ACCEL_CONFIG_2, 0x00);
	ret = IMU_WriteOneByte(dev, GYRO_SMPLRT_DIV, 0x08); // this
	ret = IMU_WriteOneByte(dev, REG_BANK_SEL, 0x00); 	//go back to bank 0
	HAL_Delay(10);
	return 0;

}
HAL_StatusTypeDef IMU_ReadOneByte(IMU_Data *dev, uint8_t reg, uint8_t *data)
{
	ret=HAL_I2C_Mem_Read(dev->i2cHandle, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
	return ret;
}


HAL_StatusTypeDef IMU_WriteOneByte(IMU_Data *dev, uint8_t reg, uint8_t data)
{
	 uint8_t regData = data;
	 return HAL_I2C_Mem_Write(dev->i2cHandle, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &regData, 1, HAL_MAX_DELAY);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {

    if (hi2c->Instance == I2C1) {
        // DMA transfer completed
    	dma_transfer_complete = 2;
    }
}

void turnLeft(float targetAngle, int dirBack){
	float finalAngle = targetAngle;
  htim1.Instance->CCR4 = 90;
	encoderL = 0;
	encoderR = 0;
	int left=0, right=0;
  HAL_Delay(800); osDelay(800);
  //forward
  if (dirBack==0){
	  right=2800;left=300;
	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);   // Assuming AIN1 is forward for motor 1
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET); // Assuming AIN2 is backward for motor 1
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);   // Assuming BIN1 is forward for motor 2
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET); // Assuming BIN2 is backward for motor 2
	  while(1){
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, right);
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, left);
		  osDelay(10);
  		  if (encoderR>=(int)(finalAngle*23.22*0.9)){
  			  right=1000;left=0;
  		  }
  		  if (encoderR>=(int)(finalAngle*23.22))break;
  	  }
  }
  //backward
  else if (dirBack==1){
	  right=2600;left=300;
	  HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // Assuming AIN1 is forward for motor 1
	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);   // Assuming AIN2 is backward for motor 1
	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // Assuming BIN1 is forward for motor 2
	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);   // Assuming BIN2 is backward for motor 2
  	  while(1){
  		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, right);
  		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, left);
  		  osDelay(10);
  		  if (encoderR>=(int)(finalAngle*20.57*0.9)){
  			  right=1000;left=0;
  		  }
  		  if (encoderR>=(int)(finalAngle*20.57)) break;
  	  }
  }
        htim1.Instance->CCR4 = 143;
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, 0);
}
void turnRight(float targetAngle, int dirBack){
	float finalAngle = targetAngle;
	encoderL = 0;
	encoderR = 0;
	int left=0, right=0;
  htim1.Instance->CCR4 = 210;
  HAL_Delay(800); osDelay(800);
  //forward
    if (dirBack==0){
    	int left=2800, right=300;
      HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);   // Assuming AIN1 is forward for motor 1
  	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET); // Assuming AIN2 is backward for motor 1
  	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);   // Assuming BIN1 is forward for motor 2
  	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET); // Assuming BIN2 is backward for motor 2
	  while(1){
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, right);
		  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, left);
		  osDelay(10);
  		  if (encoderL>=(int)(finalAngle*22.25*0.9)){
  			  right=0;left=900;
  		  }
  		  if (encoderL>=(int)(finalAngle*22.25))break;
  	  }
    }
    //backward
    else if (dirBack==1){
    	int left=2600, right=300;
      HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // Assuming AIN1 is forward for motor 1
  	  HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);   // Assuming AIN2 is backward for motor 1
  	  HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // Assuming BIN1 is forward for motor 2
  	  HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);   // Assuming BIN2 is backward for motor 2
 	  while(1){
 	  	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, right);
 	  	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, left);
 		  osDelay(10);
  		  if (encoderL>=(int)(finalAngle*21.88*0.9)){
  			  right=0;left=900;
  		  }
 		  if (encoderL>=(int)(targetAngle*21.88))break;
 	  }
    }
	        htim1.Instance->CCR4 = 130;
	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, 0);
}

void maintainStraightPath(int dista){
    // Initialize variables
	htim1.Instance->CCR4 = 139;
    uint8_t uart_buf[20];
    int16_t Data;
    int tempval = 0;
    float ratio = 0.1;
	encoderL = 0; encoderR = 0;
    int setpoint = yawAngle;
    int delta = encoderL - encoderR;
	int totravel = dista/21.0*1550.0;

	//special case for dista <= 10
	if (dista<=10 && motionCommand=='s'){
		while(1){
			pwmValL = 1330; pwmValR = 1275;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1275); // Set PWM for motor 1 (adjust value as needed)
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1330); // Set PWM for motor 2 (adjust value as needed)
			if (2*encoderR>totravel)break;
		}
	    pwmVal_servo = 139;
	    return;
	}
	else if (dista<=10 && motionCommand=='w'){
		while(1){
			pwmValL = 1260; pwmValR = 1260;
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1260); // Set PWM for motor 1 (adjust value as needed)
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1260); // Set PWM for motor 2 (adjust value as needed)
			if (2*encoderR>totravel)break;
		}
	    pwmVal_servo = 139;
	    return;
	}

	else{
	    // Main loop to maintain straight path
	    while (1) { // Use an appropriate condition to break out of the loop when needed

	    	//maintain straight line
	    	float correction = adjustServoToCenter(yawAngle, setpoint);
	    	htim1.Instance->CCR4 = (motionCommand=='w') ? 139+1.005*correction : 139-1.005*correction;

	    	//accel (nnnneeeeoooowwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww)
	    	if ((motionCommand=='w') && totravel - 2*encoderR > (int)((float)totravel*0.3)){
	    		if (ratio<1.55)ratio+=0.0003;
	            // Apply PWM to both motors to start moving forward
	    		pwmValL = (int)((float)(RPM_LEFT+20)*ratio); pwmValR = (int)((float)(RPM_RIGHT+55)*ratio);
	            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValR); // Set PWM for motor 1 (adjust value as needed)
	            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValL); // Set PWM for motor 2 (adjust value as needed)
	    	}
	    	else if (motionCommand=='w'){
		    	while(2*encoderR<totravel-222){
	            	ratio=0.8*(float)(totravel - 2*encoderR)/(float)totravel;
	    			// Apply PWM to both motors to start moving forward
	            	pwmValL = (int)((float)(RPM_LEFT+20)*ratio+900.0); pwmValR = (int)((float)(RPM_RIGHT+55)*ratio+900.0);
	    			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValR); // Set PWM for motor 1 (adjust value as needed)
	    			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValL); // Set PWM for motor 2 (adjust value as needed)
	    			osDelay(50);
	    		}
	    		break;
	    	}

	    	if ((motionCommand=='s') && totravel - 2*encoderR > (int)((float)totravel*0.3)){
	    		if (ratio<1.55)ratio+=0.0003;
	            // Apply PWM to both motors to start moving forward
	    		pwmValL = (int)((float)(RPM_LEFT+20)*ratio); pwmValR = (int)((float)(RPM_RIGHT+50)*ratio);
	            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValR); // Set PWM for motor 1 (adjust value as needed)
	            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValL); // Set PWM for motor 2 (adjust value as needed)
	    	}
	    	else if (motionCommand=='s'){
	            while(2*encoderR<totravel-222){
	            	ratio=0.8*(float)(totravel - 2*encoderR)/(float)totravel;
	    			// Apply PWM to both motors to start moving forward
		    		pwmValL = (int)((float)(RPM_LEFT+20)*ratio+900.0); pwmValR = (int)((float)(RPM_RIGHT+50)*ratio+900.0);
	    			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValR); // Set PWM for motor 1 (adjust value as needed)
	    			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValL); // Set PWM for motor 2 (adjust value as needed)
	    			osDelay(50);
	    	    	tempval = encoderR;
	    		}
	    		break;
	    	}
	    }
	}


    pwmVal_servo = 139;
    return;
}

float adjustServoToCenter(float currentAngle, float targetAngle) {
	uint8_t uart_buf[20];
    float error = currentAngle - targetAngle; // Calculate error
    if (fabs(error)<0.05){
    	error=0;
    }

	return error;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
  /* USER CODE BEGIN motor */
	uint8_t print2[20];
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // servo motor
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // start up motor 1
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // start up motor 2

	htim1.Instance->CCR4 = 139;

	int initialPwmValue = 500;  // Example initial PWM value, adjust as needed
	HAL_UART_Receive_IT(&huart3, uartRxBuffer, 5);
	 // Set initial PWM value for a smooth start
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, initialPwmValue);  // Apply to Motor 1
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, initialPwmValue);  // Apply to Motor 2
	uint8_t *status = IMU_Initialise(&imu,&hi2c1,&huart3);
	uint8_t uartbuf [20];

	//  /* Infinite loop */
	 for (;;) {
				  if (commandReceivedFlag) {
					  encoderL = 0;
					  encoderR = 0;
					commandReceivedFlag = 0; // Clear the flag
		            switch (motionCommand) {
		            	case 's':
		                	// check dirCommand
		                	if (dirCommand!='x')break;
		                	htim1.Instance->CCR4 = 139; // Center
		                	osDelay(500);

		            		// Set motor direction to backward
							// Setting GPIO Pins
		            		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET); // Assuming AIN1 is forward for motor 1
		            		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);   // Assuming AIN2 is backward for motor 1
		            		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET); // Assuming BIN1 is forward for motor 2
		            		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);   // Assuming BIN2 is backward for motor 2

		            		// Apply PWM to both motors to start moving backward
		            		pwmValL = (RPM_LEFT+295)*0.1; pwmValR = (RPM_RIGHT+35)*0.1;
		            		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (RPM_RIGHT+35)*0.1); // Set PWM for motor 1 (adjust value as needed)
		            		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (RPM_LEFT+295)*0.1); // Set PWM for motor 2 (adjust value as needed)

		            		maintainStraightPath(distCommand);

		                	// Motor Stop
		                	// Apply PWM to both motors to stop moving
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0); // Set PWM for motor 1 (adjust value as needed)
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0); // Set PWM for motor 2 (adjust value as needed)
		                    pwmValL = 0; pwmValR = 0;
		                    HAL_UART_Transmit(&huart3, "ACK", 3, HAL_MAX_DELAY);
			                    HAL_Delay(100);
							break;
		                case 'w':
		                	// check dirCommand
		                	if (dirCommand!='x')break;
		                	htim1.Instance->CCR4 = 139; // Center
		                	osDelay(500);

		                	// Set motor direction to forward
							// Setting GPIO Pins
		                    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);   // Assuming AIN1 is forward for motor 1
		                    HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET); // Assuming AIN2 is backward for motor 1
		                    HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);   // Assuming BIN1 is forward for motor 2
		                    HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET); // Assuming BIN2 is backward for motor 2

		                    // Apply PWM to both motors to start moving forward
		                    pwmValL = (RPM_LEFT)*0.1; pwmValR = (RPM_RIGHT+77)*0.1;
		                    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValR); // Set PWM for motor 1 (adjust value as needed)
		                    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValL); // Set PWM for motor 2 (adjust value as needed)

		                    maintainStraightPath(distCommand);
		                	// Motor Stop
		                	// Apply PWM to both motors to stop moving
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0); // Set PWM for motor 1 (adjust value as needed)
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0); // Set PWM for motor 2 (adjust value as needed)
							pwmValL = 0; pwmValR = 0;

		                    HAL_UART_Transmit(&huart3, "ACK", 3, HAL_MAX_DELAY);

		                    HAL_Delay(100);
		                    break;

		                case 't':
		                	htim1.Instance->CCR4 = 139;
		                	osDelay(500);
		                	if (dirCommand=='a'){
		                		htim1.Instance->CCR4 = 90;
		                	}
		                	else if (dirCommand=='d'){
		                		  htim1.Instance->CCR4 = 210;
		                	}
		                	break;


		                case 'f': // forward angle rotation
		                	// check left/right
		                	if (dirCommand=='a'){
		                		turnLeft(distCommand,0);
		                		HAL_UART_Transmit(&huart3, "ACK", 3, HAL_MAX_DELAY);
		                		HAL_Delay(100);
		                	}
		                	else if (dirCommand=='d'){
		                		turnRight(distCommand,0);

		                		HAL_UART_Transmit(&huart3, "ACK", 3, HAL_MAX_DELAY);
		                		HAL_Delay(100);
		                	}
							break;

		                case 'b': // backward angle rotation
		                	// check left/right
		                	if (dirCommand=='a'){
		                		turnLeft(distCommand,1);
		                		HAL_UART_Transmit(&huart3, "ACK", 3, HAL_MAX_DELAY);
		                		HAL_Delay(100);
		                	}
		                	else if (dirCommand=='d'){
		                		turnRight(distCommand,1);
		                		HAL_UART_Transmit(&huart3, "ACK", 3, HAL_MAX_DELAY);
		                		HAL_Delay(100);
		                	}
							break;

		                default:
		                	// Motor Stop
		                	// Apply PWM to both motors to stop moving
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0); // Set PWM for motor 1 (adjust value as needed)
							__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0); // Set PWM for motor 2 (adjust value as needed)

		                    break;

		        }
		    }
			osDelay(1000);

		  }
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_encoder */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Start encoder for Motor 1
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Start encoder for Motor 2

    int cnt_motor1=0,cnt_motor2=0;


    __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset counter for Motor 1
    __HAL_TIM_SET_COUNTER(&htim3, 0); // Reset counter for Motor 2
    uint32_t tick = HAL_GetTick();

    /* Infinite loop */
    for (;;)
    {
        if (HAL_GetTick() - tick > 5L) // Update every 10 milliseconds
        {
            //motor 1 (R)
        	cnt_motor1 = __HAL_TIM_GET_COUNTER(&htim2);
        	__HAL_TIM_SET_COUNTER(&htim2, 0); // Reset counter for Motor 1
            if (cnt_motor1>32000){
            	//backwards (overflow)
            	encoderR += 0.5*(65536 - cnt_motor1);
            }
            else{
            	//forwards
            	encoderR += 0.5*cnt_motor1;
            }

            //motor 2 (L)
        	cnt_motor2 = __HAL_TIM_GET_COUNTER(&htim3);
        	__HAL_TIM_SET_COUNTER(&htim3, 0); // Reset counter for Motor 2
            if (cnt_motor2>32000){
            	//backwards (overflow)
            	encoderL += 65536 - cnt_motor2;
            }
            else{
            	//forwards
            	encoderL += cnt_motor2;
            }

            tick = HAL_GetTick();
        }
    }
  /* USER CODE END encoder */
}

/* USER CODE BEGIN Header_Gyro */
/**
* @brief Function implementing the GyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gyro */
void Gyro(void *argument)
{
  /* USER CODE BEGIN Gyro */
	  uint8_t uart_buf[20];
	  prev_time_elapsed = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {
    	  if(dma_transfer_complete==0){
              ret = HAL_I2C_Mem_Read_DMA(imu.i2cHandle, IMU_ADDR,GYRO_ZOUT_H,I2C_MEMADD_SIZE_8BIT,rawData,2);
              if(ret == HAL_OK)
              dma_transfer_complete = 2;
            }
          if (dma_transfer_complete == 2) {
    			time_elapsed = HAL_GetTick();
    			time_difference = time_elapsed - prev_time_elapsed; // Calculate time difference between readings
    			Data = (int16_t)(rawData[0] << 8 | rawData[1]); // Combine high and low bytes

    			// Calculate new yaw angle based on gyro data
    			yawAngle += (float)((time_difference * (Data - 240.0) / 131.0)) / 1000.0;

    			// Offsetting the drift (derived empirically)
    			yawAngle -= (float)(time_difference*0.075/1000.0);

           dma_transfer_complete = 0;
           prev_time_elapsed = time_elapsed;
          }

  }
  /* USER CODE END Gyro */
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
