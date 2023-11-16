/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "ICM20948.h"
//	#include "pid.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId MotorTaskHandle;
osThreadId EncoderTaskHandle;
osThreadId gyroTestHandle;
osThreadId servoTaskHandle;
osThreadId IRTaskHandle;
osThreadId turningTaskHandle;
osThreadId ultrasonciTaskHandle;
osThreadId task2Handle;
/* USER CODE BEGIN PV */
//	Start or stop the robot
uint8_t currently_running;

//	Successful or unsuccessful run
uint8_t success[2] = "0";
uint8_t fail[2] = "1";

//	IR Sensors
int stopDist = 0;
uint8_t stopDistStr[10];
uint8_t IR_Dist_1;
uint8_t IR_Dist_2;
uint8_t IR_Dist_1_Str[20];
uint8_t IR_Dist_2_Str[20];

//	Ultrasonic
int Is_First_Captured = 0;
int32_t IC_Val1 = 0;
int32_t IC_Val2 = 0;
double Difference = 0;
double Distance = 0;

//	How far the car must travel
int dist_to_travel;
float tempDist = 0; //	Used to calculate how far the car has travelled

//	Angle of servomotor
double center = 75; //	Center
double motor_angle = 75;
//double max_right = 100;
double max_right = 120;
double max_left = 51;
float target_angle = 0; // Target angle from rpi (I think)

//	Gyroscope value to track angle
float yawG;
uint8_t yawStr[20];

//	Current PWM values
float pwm_val_L = 0;
float pwm_val_R = 0;

//	Direction: forwards or backwards
int motor_dir;

//	For UART transmit purposes
uint8_t aRxBuffer[20];
uint8_t tempBuf[2];

//	Robot is trying to move straight if variable is set to 1, else trying to turn
int movingStraight = 1;

int turningRight = 0;
int turningLeft = 0;

//	Flag to start task 2
int startTask2 = 0;

//	Scenarios for task 2 obstacle 1
int scenarioL = 0;
uint8_t L_Instructions[10][20] = {"tl50", "tr53", "tr50", "tl47" };
uint8_t R_Instructions[10][20] = {"tr50", "tl50", "tl45","tr48" };
int index = 0;
int scenarioR = 0;

int firstTurnLength = 750;
int turn90xDisplacement = 250;

//	Flag to move forward after first obstacle
int secondStraight = 0;

//	Scenarios for task 2 obstacle 1
uint8_t L_Instructions_2[10][20] = { "tl82", "mb300", "mf10000", "tr175",
		"mf10000", "tr90" };
uint8_t R_Instructions_2[10][20] = { "tr90", "mb300", "mf10000", "tl156",
		"mf10000", "tl81" };
int scenarioL_2 = 0;
int scenarioR_2 = 0;
int index_2 = 0;

//	Determine when returning to the carpark, whether the car is to the left or right
//	0 for left and 1 or right;
int carLocation = -1;

//	Returning to the carpark
int returnCarPark = 0;
int finalTurns = 0;

//	Encoder distance travelled
float distA = 0;

//	Track current position of the bot
int currentMov = 0;

//	Reset all variables for task 2 testing purposes
int manualReset = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const *argument);
void motors(void const *argument);
void encoder_task(void const *argument);
void gyroRead(void const *argument);
void servoMotor(void const *argument);
void IRFunc(void const *argument);
void turningFunc(void const *argument);
void ultrasonic(void const *argument);
void obstacleTask(void const *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM8_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();
	MX_USART3_UART_Init();
	MX_ADC2_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	OLED_Init();

	//	Receive 5 characters
//	HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,5);
	HAL_UART_Receive_IT(&huart3, tempBuf, 1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	/* USER CODE END 2 */

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
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of MotorTask */
	osThreadDef(MotorTask, motors, osPriorityIdle, 0, 128);
	MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

	/* definition and creation of EncoderTask */
	osThreadDef(EncoderTask, encoder_task, osPriorityIdle, 0, 512);
	EncoderTaskHandle = osThreadCreate(osThread(EncoderTask), NULL);

	/* definition and creation of gyroTest */
	osThreadDef(gyroTest, gyroRead, osPriorityIdle, 0, 256);
	gyroTestHandle = osThreadCreate(osThread(gyroTest), NULL);

	/* definition and creation of servoTask */
	osThreadDef(servoTask, servoMotor, osPriorityIdle, 0, 128);
	servoTaskHandle = osThreadCreate(osThread(servoTask), NULL);

	/* definition and creation of IRTask */
	osThreadDef(IRTask, IRFunc, osPriorityIdle, 0, 256);
	IRTaskHandle = osThreadCreate(osThread(IRTask), NULL);

	/* definition and creation of turningTask */
//  osThreadDef(turningTask, turningFunc, osPriorityIdle, 0, 128);
//  turningTaskHandle = osThreadCreate(osThread(turningTask), NULL);
	/* definition and creation of ultrasonciTask */
	osThreadDef(ultrasonciTask, ultrasonic, osPriorityIdle, 0, 256);
	ultrasonciTaskHandle = osThreadCreate(osThread(ultrasonciTask), NULL);

	/* definition and creation of task2 */
	osThreadDef(task2, obstacleTask, osPriorityIdle, 0, 256);
	task2Handle = osThreadCreate(osThread(task2), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 320;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
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
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

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
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 16;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 7199;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
	OLED_SCL_Pin | OLED_SDA_Pin | OLED_RST_Pin | OLED_DC_Pin | LED3_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin | AIN1_Pin | BIN1_Pin | BIN2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ultrasonic_GPIO_Port, ultrasonic_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
	 LED3_Pin */
	GPIO_InitStruct.Pin = OLED_SCL_Pin | OLED_SDA_Pin | OLED_RST_Pin
			| OLED_DC_Pin | LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
	GPIO_InitStruct.Pin = AIN2_Pin | AIN1_Pin | BIN1_Pin | BIN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_BUTTON_Pin */
	GPIO_InitStruct.Pin = USER_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ultrasonic_Pin */
	GPIO_InitStruct.Pin = ultrasonic_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ultrasonic_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//	Prevent unused argument(s) compilation warning
	UNUSED(huart);

	//	If ENTER key is pressed
	if (tempBuf[0] == '\n' || tempBuf[0] == '\r') {
		checkBits();
//		sprintf(aRxBuffer,"\n\r");
//		HAL_UART_Transmit(&huart3, (uint8_t*) aRxBuffer, strlen(aRxBuffer), 0xFFFF);
		sprintf(aRxBuffer, "");
	}

	else {
		sprintf(aRxBuffer, "%s%c", aRxBuffer, tempBuf[0]);
//		HAL_UART_Transmit(&huart3, (uint8_t*) tempBuf, 1, 0xFFFF);
	}
	HAL_UART_Receive_IT(&huart3, tempBuf, 1);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
			{
		if (Is_First_Captured == 0) // if the first value is not captured
				{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured == 1)   // if the first is already captured
				{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1) {
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2) {
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034 / 2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
		}
	}
}

void checkBits() {
//	if (aRxBuffer[0] == 'A')
//	{
//		motor_angle = max_left;
//	}
//	else if (aRxBuffer[0] == 'D' )
//	{
//		motor_angle = max_right;
//	}
//	else if (aRxBuffer[0] == 'C' )
//	{
//		motor_angle = center;
//	}
//	else if (aRxBuffer[0] == 'W' && motor_dir != 1)
//	{
//		motor_dir = 1;
//	}
//	else if (aRxBuffer[0] == 'W' && motor_dir == 1)
//	{
//		pwm_val_R += 100;
//		pwm_val_L += 100;
//	}
//	else if (aRxBuffer[0] == 'S' && motor_dir != 0)
//	{
//		motor_dir = 0;
//	}
//	else if (aRxBuffer[0] == 'S' && motor_dir == 0)
//	{
//		pwm_val_R += 100;
//		pwm_val_L += 100;
//	}
//	else if (aRxBuffer[0] == 'K' )
//	{
//		if (pwm_val_R > 0 || pwm_val_L > 0)
//		{
//			pwm_val_R = 0;
//			pwm_val_L = 0;
//		}
//		else
//		{
//			pwm_val_R = 1000;
//			pwm_val_L = 1000;
//		}
//	}
	if (aRxBuffer[0] == 'm') {
		if (aRxBuffer[1] == 'f') {
			motor_angle = center;
			movingStraight = 1;
			motor_dir = 1;
			dist_to_travel = getNumFromArr(2);
			pwm_val_L = 4000;
			pwm_val_R = 4000;
		} else if (aRxBuffer[1] == 'b') {
			movingStraight = 1;
			motor_angle = center;
			motor_dir = 0;
			dist_to_travel = getNumFromArr(2);
			pwm_val_L = 4000;
			pwm_val_R = 4000;
		}
	} else if (aRxBuffer[0] == 't') {
		if (aRxBuffer[1] == 'r') {
			motor_dir = 1;
			turningRight = 1;
			turningLeft = 0;
			movingStraight = 0;
			motor_angle = max_right;
			target_angle = getNumFromArr(2) * 0.99;
			pwm_val_L = 3000;
//			pwm_val_R = 2000 * 43 / 61;
			pwm_val_R = 1500;
		}
		if (aRxBuffer[1] == 'l') {
			motor_dir = 1;
			turningRight = 0;
			turningLeft = 1;
			movingStraight = 0;
			motor_angle = max_left;
			target_angle = getNumFromArr(2) * 0.99;
			pwm_val_L = 1500;
			pwm_val_R = 3000;
		}
	} else if (aRxBuffer[0] == 'b') {
		if (aRxBuffer[1] == 'r') {
			motor_dir = 0;
			turningRight = 1;
			turningLeft = 0;
			movingStraight = 0;
			motor_angle = max_right;
			target_angle = getNumFromArr(2) * 0.99;
			pwm_val_L = 3000;
			pwm_val_R = 1500;
		}
		if (aRxBuffer[1] == 'l') {
			motor_dir = 0;
			turningRight = 0;
			turningLeft = 1;
			movingStraight = 0;
			motor_angle = max_left;
			target_angle = getNumFromArr(2) * 0.99;
			pwm_val_L = 1500;
			pwm_val_R = 3000;
		}
	} else if (aRxBuffer[0] == 's' && aRxBuffer[1] == 't') {
		motor_dir = 2;
		stopDist = getNumFromArr(2);
		stopDist /= 10;
//		if (IR_Dist_1 < IR_Dist_2)
//		{
//			HAL_UART_Transmit(&huart3, (uint8_t*) IR_Dist_1_Str, 2, 0xFFFF);
//		}
//		else
//		{
//			HAL_UART_Transmit(&huart3, (uint8_t*) IR_Dist_2_Str, 2, 0xFFFF);
//		}
//		if (IR_Dist_1 <= stopDist || IR_Dist_2 <= stopDist)
//		{
//			stopDist = 0;
//		}
	} else if (aRxBuffer[0] == 'l') {
		scenarioL = 1;
	} else if (aRxBuffer[0] == 'r') {
		scenarioR = 1;
	} else if (aRxBuffer[0] == 'L') {
		scenarioL_2 = 1;
		carLocation = 0;
	} else if (aRxBuffer[0] == 'R') {
		scenarioR_2 = 1;
		carLocation = 1;
	} else if (aRxBuffer[0] == '1') {
		startTask2 = 1;
		stopDist = 20;
		motor_dir = 1;
	} else if (aRxBuffer[0] == '2') {
		secondStraight = 1;
		stopDist = 30;
		motor_dir = 1;
	} else if (aRxBuffer[0] == '3') {
		returnCarPark = 1;
	} else if (aRxBuffer[0] == '4') {
		finalTurns = 1;
	} else {
		tempDist = 0;
		stopDist = 0;
		movingStraight = 1;
		motor_dir = 1;
		dist_to_travel = 0;
		target_angle = 0;
		motor_angle = center;
		turningRight = 0;
		turningLeft = 0;
		pwm_val_L = 0;
		pwm_val_R = 0;
		startTask2 = 0;
		secondStraight = 0;
		scenarioL = 0;
		scenarioR = 0;
		scenarioL_2 = 0;
		scenarioR_2 = 0;
		returnCarPark = 0;
		index = 0;
		index_2 = 0;
		carLocation = -1;
		returnCarPark = 0;
		finalTurns = 0;
		currentMov = 0;
		manualReset = 1;
//		HAL_UART_Transmit(&huart3, (uint8_t*) fail, 1, 0xFFFF);
	}
}

void stopAll() {
	tempDist = 0;
//	stopDist = 0;
	movingStraight = 1;
	motor_dir = 1;
	dist_to_travel = 0;
	target_angle = 0;
	motor_angle = center;
	turningRight = 0;
	turningLeft = 0;
	pwm_val_L = 0;
	pwm_val_R = 0;
	startTask2 = 0;
	secondStraight = 0;
	scenarioL = 0;
	scenarioR = 0;
	scenarioL_2 = 0;
	scenarioR_2 = 0;
	returnCarPark = 0;
	index = 0;
	index_2 = 0;
//	carLocation = -1;
	returnCarPark = 0;
	finalTurns = 0;
}

int getNumFromArr(int index) {
	int result = 0;
	while (aRxBuffer[index] >= '0' && aRxBuffer[index] <= '9') {
		result = result * 10 + (aRxBuffer[index] - '0');
		index++;
	}
	return result;
}

void delay_us(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER (&htim4) < time)
		;
}

void Ultrasonic_Read(void) {
	// Code for Ultrasonic Sensor
	HAL_GPIO_WritePin(ultrasonic_GPIO_Port, ultrasonic_Pin, GPIO_PIN_SET); // pull the TRIG pin HIGH
	delay_us(10);
	HAL_GPIO_WritePin(ultrasonic_GPIO_Port, ultrasonic_Pin, GPIO_PIN_RESET); // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	uint8_t pressed = 0;
	uint8_t running[20] = "RUNNING";
	uint8_t pause[20] = "PAUSE...";
	/* Infinite loop */
	for (;;) {
		if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin)
				== GPIO_PIN_SET) {
			pressed = 0;
		}
		if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin)
				== GPIO_PIN_RESET && pressed == 0) {
			currently_running++;
			currently_running %= 3;
			pressed = 1;
		}
		if (currently_running) {
			OLED_ShowString(10, 0, running);
		} else {
			OLED_ShowString(10, 0, pause);
		}
		OLED_Refresh_Gram();
//		if (currently_running == 2 && !ran)
//		{
//			motor_dir = 1;
//			turningRight = 0;
//			turningLeft = 1;
//			movingStraight = 0;
//			motor_angle = max_left;
//			target_angle = 188;
//			pwm_val_L = 2000;
//			pwm_val_R = 2000;
//			ran = 1;
//
//		}
		//	Display the uart transmissions on the OLED
//		if (aRxBuffer == "\0")
//		{
//			OLED_Clear();
//		}
//		else
//		{
//			OLED_ShowString(10,10,aRxBuffer);
//		}
//		OLED_Refresh_Gram();
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motors */
/**
 * @brief Function implementing the MotorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_motors */
void motors(void const *argument) {
	/* USER CODE BEGIN motors */

	// MotorA (the one on the left)
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	// MotorB (the one on the right)
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

//	pwm_val_L = 2000;
//	pwm_val_R = 2000;

	/* Infinite loop */

	for (;;) {
		if (currently_running == 2) {
//			if (turningRight == 1)
//			{
//				HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwm_val_L); // Modify the comparison value from duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwm_val_R);
//			}
//			else if (turningLeft == 1)
//			{
//				HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwm_val_L); // Modify the comparison value from duty cycle
//				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwm_val_R);
//			}
			if (motor_dir == 1) {
				HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwm_val_L); // Modify the comparison value from duty cycle
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwm_val_R);
			} else {
				HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwm_val_L); // Modify the comparison value from duty cycle
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwm_val_R);
			}
		} else {
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0); // Modify the comparison value from duty cycle
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		}

	}
	/* USER CODE END motors */
}

/* USER CODE BEGIN Header_encoder_task */
/**
 * @brief Function implementing the EncoderTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_encoder_task */
void encoder_task(void const *argument) {
	/* USER CODE BEGIN encoder_task */
	/* Infinite loop */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // MotorA
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // MotorB

	int cnt1A, cnt2A, diffA;
	int cnt1B, cnt2B, diffB;

//	float distA = 0;
//	float distB = 0;

//	float tempDist = 0;

	float curGyro = 0;

	uint8_t helloA[20];
	uint8_t helloB[20];

	uint16_t dirA;
	uint16_t dirB;

	float newTime, oldTime, dT, speedA, speedB;
	newTime = HAL_GetTick();
	oldTime = 0.0;

	cnt1A = __HAL_TIM_GET_COUNTER(&htim2);
	cnt1B = __HAL_TIM_GET_COUNTER(&htim3);

	for (;;) {
		if (currently_running == 2) {
			if (HAL_GetTick() - newTime > 200L) {
				cnt2A = __HAL_TIM_GET_COUNTER(&htim2);
				cnt2B = __HAL_TIM_GET_COUNTER(&htim3);

				// MotorA
				if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
					if (cnt2A < cnt1A) {
						diffA = cnt1A - cnt2A;
					} else {
						diffA = (65535 - cnt2A) + cnt1A;
					}
				} else {
					if (cnt2A > cnt1A) {
						diffA = cnt2A - cnt1A;
					} else {
						diffA = (65535 - cnt1A) + cnt2A;
					}
				}

				// MotorB
				if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
					if (cnt2B < cnt1B) {
						diffB = cnt1B - cnt2B;
					} else {
						diffB = (65535 - cnt2B) + cnt1B;
					}
				} else {
					if (cnt2B > cnt1B) {
						diffB = cnt2B - cnt1B;
					} else {
						diffB = (65535 - cnt1B) + cnt2B;
					}
				}

//				sprintf(helloA, "Speed A:%5d\0", diffA);
//				OLED_ShowString(10, 20, helloA);
//
//				dirA = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
//				sprintf(helloA, "Dir A:%5d\0", dirA);
//				OLED_ShowString(10, 30, helloA);
//
//				sprintf(helloB, "Speed B:%5d\0", diffB);
//				OLED_ShowString(10, 40, helloB);
//
//				dirB = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
//				sprintf(helloB, "Dir B:%5d\0", dirB);
//				OLED_ShowString(10, 50, helloB);

				cnt1A = __HAL_TIM_GET_COUNTER(&htim2);
				cnt1B = __HAL_TIM_GET_COUNTER(&htim3);

//				At max_right (100), half a circle is 1420 (based on Motor A)
//				At max_left (55), half a circle is 1150 (based on Motor B)

				if (diffA < 10000 && diffB < 10000) {
					newTime = HAL_GetTick();

					dT = newTime - oldTime;
					oldTime = newTime;

					distA += diffA * 0.68227 * dT * 0.001;
					sprintf(helloA, "Dist:%3.2f          \0", distA);
//					OLED_ShowString(10, 20, helloA);

//					distB += diffB * 0.68227 * dT * 0.001;
//					sprintf(helloA, "Dist B:%3.2f        \0", distB);
//					OLED_ShowString(10, 40, helloA);

					//	For right turn
					//	For back wheel, radius = 1500
					//	For center, radius = 1450
//					if (distA >= (1450 * target_angle / 180) && movingStraight != 1)
//					{
//						pwm_val_R = 0;
//						pwm_val_L = 0;
//					}

					//	For left turn
					//	For back wheel, radius = 1060
					//	For center, radius = 950
//					if (distA >= (950 * target_angle / 180) && movingStraight != 1)
//					{
//						pwm_val_R = 0;
//						pwm_val_L = 0;
//					}

					if (target_angle != 0 && !movingStraight
							&& motor_dir == 1) {
						if (tempDist == 0) {
							tempDist = distA;
						}
						if (turningRight
								&& (distA - tempDist
										>= (1210 * target_angle / 180)))
//						if (turningRight && (distA - tempDist >= (1450 * target_angle / 180)))
//						if (turningRight && (distA - tempDist >= (3000 * target_angle / 180)))
								{
							target_angle = 0;
							tempDist = 0;
							pwm_val_R = 0;
							pwm_val_L = 0;
							motor_angle = center;
							turningRight = 0;
//							HAL_UART_Transmit(&huart3, (uint8_t*) success, 1,
//									0xFFFF);
						} else if (turningRight
								&& (distA - tempDist
										>= ((1210 * target_angle / 180) - 150))) {
							pwm_val_R = 500;
							pwm_val_L = 1000;
						} else if (turningLeft
								&& (distA - tempDist
										>= (665 * target_angle / 180))) {
							target_angle = 0;
							tempDist = 0;
							pwm_val_R = 0;
							pwm_val_L = 0;
							motor_angle = center;
							turningLeft = 0;
//							HAL_UART_Transmit(&huart3, (uint8_t*) success, 1,
//									0xFFFF);
						} else if (turningLeft
								&& (distA - tempDist
										>= ((665 * target_angle / 180) - 80))) {
							pwm_val_R = 1000;
							pwm_val_L = 500;
						}
					}

					if (target_angle != 0 && !movingStraight
							&& motor_dir == 0) {
						if (tempDist == 0) {
							tempDist = distA;
						}
						if (turningRight
								&& (distA - tempDist
										>= (1210 * target_angle / 180)))
//						if (turningRight && (distA - tempDist >= (3000 * target_angle / 180)))
								{
							target_angle = 0;
							tempDist = 0;
							pwm_val_R = 0;
							pwm_val_L = 0;
							motor_angle = center;
							turningRight = 0;
//							HAL_UART_Transmit(&huart3, (uint8_t*) success, 1,
//									0xFFFF);
						} else if (turningRight
								&& (distA - tempDist
										>= ((1210 * target_angle / 180) - 150))) {
							pwm_val_R = 500;
							pwm_val_L = 1000;
						} else if (turningLeft
								&& (distA - tempDist
										>= (660 * target_angle / 180))) {
							target_angle = 0;
							tempDist = 0;
							pwm_val_R = 0;
							pwm_val_L = 0;
							motor_angle = center;
							turningLeft = 0;
//							HAL_UART_Transmit(&huart3, (uint8_t*) success, 1,
//									0xFFFF);
						} else if (turningLeft
								&& (distA - tempDist
										>= ((660 * target_angle / 180) - 80))) {
							pwm_val_R = 1000;
							pwm_val_L = 500;
						}
					}

					//	For straight line distance
//					if (dist_to_travel != 0 && distA >= dist_to_travel && movingStraight == 1)
//					{
//						dist_to_travel = 0;
//						pwm_val_R = 0;
//						pwm_val_L = 0;
//					}
					if (dist_to_travel != 0 && movingStraight) {
						if (tempDist == 0) {
							tempDist = distA;
						}
						if (distA - tempDist >= dist_to_travel) {
							dist_to_travel = 0;
							tempDist = 0;
							pwm_val_R = 0;
							pwm_val_L = 0;
//							HAL_UART_Transmit(&huart3, (uint8_t*) success, 1,
//									0xFFFF);
						} else if (distA - tempDist >= dist_to_travel - 150) {
							pwm_val_R = 1000;
							pwm_val_L = 1000;
						}
					}

					//	Moving Straight with gyro
					if (movingStraight && dist_to_travel != 0) {
						if (curGyro == 0) {
							curGyro = yawG;
						}
						if (motor_dir == 1) {
							if (yawG < curGyro - 1.5) {
								motor_angle = 73;
							} else if (yawG > curGyro + 1.5) {
								motor_angle = 77;
							} else if (yawG > (curGyro - 1)
									&& yawG < (curGyro + 1)) {
								motor_angle = center;
							}
						} else if (motor_dir == 0) {
							if (yawG < curGyro - 1.5) {
								motor_angle = 77;
							} else if (yawG > curGyro + 1.5) {
								motor_angle = 73;
							} else if (yawG > (curGyro - 1)
									&& yawG < (curGyro + 1)) {
								motor_angle = center;
							}
						}
					} else {
						curGyro = 0;
					}
				}
				if (scenarioL) {
					if (target_angle == 0 && index < 4) {
						osDelay(400);
						sprintf(aRxBuffer, L_Instructions[index++]);
//						sprintf(aRxBuffer,"tl45");
//						index++;
						checkBits();
					} else if (index == 4 && target_angle == 0) {
						sprintf(aRxBuffer, "");
						index = 0;
						scenarioL = 0;
						currentMov = 2;
					}
				}
				if (scenarioR) {
					if (target_angle == 0 && index < 4) {
						osDelay(400);
						sprintf(aRxBuffer, R_Instructions[index++]);
						checkBits();
					} else if (index == 4 && target_angle == 0) {
						sprintf(aRxBuffer, "");
						index = 0;
						scenarioR = 0;
						currentMov = 2;
					}
				}

			}
		}
		osDelay(1);
	}

	/*
	 uint8_t helloA[20];
	 uint8_t helloB[20];

	 HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // MotorA
	 HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // MotorB

	 uint32_t startTime = HAL_GetTick();
	 uint32_t prevTime = startTime;
	 uint32_t currTime;

	 float totalDistance_left = 0;
	 float totalDistance_right = 0;

	 float distLeft, distRight;

	 int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	 int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim3);

	 int leftTick, rightTick;

	 int diffLeft = 0;
	 int diffRight = 0;


	 for (;;)
	 {
	 if (currently_running == 2)
	 {
	 currTime = HAL_GetTick();
	 if (currTime - prevTime > 60L) {
	 leftTick = __HAL_TIM_GET_COUNTER(&htim2);
	 rightTick = __HAL_TIM_GET_COUNTER(&htim3);

	 diffLeft = 0;
	 diffRight = 0;

	 if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
	 if (leftTick < leftTick_prev)
	 diffLeft = leftTick_prev - leftTick;
	 else
	 diffLeft = (65535 - leftTick) + leftTick_prev;
	 } else {
	 if (leftTick > leftTick_prev)
	 diffLeft = leftTick - leftTick_prev;
	 else
	 diffLeft = 65535 - leftTick_prev + leftTick;
	 }

	 if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
	 if (rightTick < rightTick_prev)
	 diffRight = rightTick_prev - rightTick;
	 else
	 diffRight = (65535 - rightTick) + rightTick_prev;
	 } else {
	 if (rightTick > rightTick_prev)
	 diffRight = rightTick - rightTick_prev;
	 else
	 diffRight = 65535 - rightTick_prev + rightTick;
	 }
	 distLeft = ((float) diffLeft / 333) * 20.42;
	 totalDistance_left += distLeft;

	 // right measured distance
	 distRight = ((float) diffRight / 333) * 20.42;
	 totalDistance_right += distRight;

	 prevTime = currTime;
	 leftTick_prev = leftTick;
	 rightTick_prev = rightTick;

	 sprintf(helloA, "Dist L:%5d\0", totalDistance_left);
	 OLED_ShowString(10, 30, helloA);

	 sprintf(helloB, "Dist R:%5d\0", totalDistance_right);
	 OLED_ShowString(10, 40, helloB);
	 }
	 }
	 }*/

	/* USER CODE END encoder_task */
}

/* USER CODE BEGIN Header_gyroRead */
/**
 * @brief Function implementing the gyroTest thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_gyroRead */
void gyroRead(void const *argument) {
	/* USER CODE BEGIN gyroRead */
	ICM20948 imu;
	float newTime, oldTime, dT;
	yawG = 0.0; // Initialize yaw estimation

	oldTime = 0.0;

	uint8_t running[20] = "Calibrating..";
	uint8_t done[20] = "Proceed..";
	uint8_t clear[20] = "                  ";

	int calibrated = 0;
	int signalToProceed = 0;

	/* Infinite loop */
	for (;;) {
		//	Calibrate the gyroscope by re-initialising until we get a good reading
		if (currently_running == 1 && !calibrated) {
			OLED_ShowString(10, 10, running);
			osDelay(1000);
			do {
				IMU_Initialise(&imu, &hi2c1, &huart3);

				oldTime = 0.0;
				yawG = 0.0;

				IMU_GyroRead(&imu);

				newTime = HAL_GetTick();
				dT = newTime - oldTime;
				oldTime = newTime;

				// Calculate yaw from gyroscope
				yawG = yawG + imu.gyro[2] * dT * 0.001;
			} while (abs(yawG) > 1);
			calibrated = 1;
			OLED_ShowString(10, 10, clear);
			OLED_ShowString(10, 10, done);
		}

		if (currently_running == 2) {
			if (!signalToProceed) {
//				HAL_UART_Transmit(&huart3, (uint8_t*) done, 7, 0xFFFF);
				signalToProceed = 1;
			}
			IMU_GyroRead(&imu);

			newTime = HAL_GetTick();

			dT = newTime - oldTime;
			oldTime = newTime;

			// Calculate yaw from gyroscope
			yawG = yawG + imu.gyro[2] * dT * 0.001;
			if (yawG > 360) {
				yawG -= 360;
			} else if (yawG < -360) {
				yawG += 360;
			}

			sprintf(yawStr, "Yaw: %5.2f", yawG);
			OLED_ShowString(10, 10, yawStr);
		}
	}
	/* USER CODE END gyroRead */
}

/* USER CODE BEGIN Header_servoMotor */
/**
 * @brief Function implementing the servoTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_servoMotor */
void servoMotor(void const *argument) {
	/* USER CODE BEGIN servoMotor */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	/* Infinite loop */
	htim1.Instance->CCR4 = center;
	for (;;) {
		if (currently_running == 2) {
			htim1.Instance->CCR4 = motor_angle;
		}
		//osDelay(1000);
	}
	/* USER CODE END servoMotor */
}

/* USER CODE BEGIN Header_IRFunc */
/**
 * @brief Function implementing the IRTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IRFunc */
void IRFunc(void const *argument) {
	/* USER CODE BEGIN IRFunc */
	uint32_t adcVal1, adcVal2;
	float voltage1, voltage2;
	/* Infinite loop */
	for (;;) {
		if (currently_running == 2) {
			//	Left IR Sensor
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adcVal1 = HAL_ADC_GetValue(&hadc1); // Raw data
			voltage1 = (adcVal1 / pow(2, 12)) * 3.3;
			IR_Dist_1 = 1
					/ (0.0140817 * pow(voltage1, 2) + 0.00685361 * voltage1
							+ 0.012403);
			sprintf(IR_Dist_1_Str, "%2d", IR_Dist_1);
//			sprintf(IR_Dist_1_Str, "left: %2d   ", IR_Dist_1);
//			OLED_ShowString(10, 40, IR_Dist_1_Str);

			//	Right IR Sensor
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 100);
			adcVal2 = HAL_ADC_GetValue(&hadc2); // Raw data
			voltage2 = (adcVal2 / pow(2, 12)) * 3.3;
			IR_Dist_2 = 1
					/ (0.0140817 * pow(voltage2, 2) + 0.00685361 * voltage2
							+ 0.012403);
			sprintf(IR_Dist_2_Str, "%2d", IR_Dist_2);
//			sprintf(IR_Dist_2_Str, "right: %2d\n\r", IR_Dist_2);
//			OLED_ShowString(10, 50, IR_Dist_2_Str);

//			HAL_UART_Transmit(&huart3, (uint8_t*) IR_Dist_1_Str, strlen(IR_Dist_1_Str), 0xFFFF);
//			HAL_UART_Transmit(&huart3, (uint8_t*) IR_Dist_2_Str, strlen(IR_Dist_2_Str), 0xFFFF);
//			if (stopDist != 0 && motor_dir == 1)
//			{
//				if (IR_Dist_1 <= stopDist || IR_Dist_2 <= stopDist)
//				{
//					stopDist = 0;
//					movingStraight = 1;
//					dist_to_travel = 0;
//					target_angle = 0;
//					motor_angle = center;
//					turningRight = 0;
//					turningLeft = 0;
//					pwm_val_L = 0;
//					pwm_val_R = 0;
//					tempDist = 0;
//					HAL_UART_Transmit(&huart3, (uint8_t*) success, 1, 0xFFFF);
//				}
//			}
//			else if (stopDist != 0 && motor_dir == 0)
//			{
//				stopDist = 0;
//			}

			//	Robot is trying to move forward
			//	Normal obstacle avoidance
//			if (target_angle == 0){
//				if (IR_Dist_1 <= 10 && IR_Dist_2 <= 10){
//					motor_dir = 0;
//					motor_angle = max_right;
//				}
//				else if (IR_Dist_1 <= 10 && IR_Dist_2 > 10){
//					motor_dir = 0;
//					motor_angle = max_left;
//				}
//				else if (IR_Dist_1 > 10 && IR_Dist_2 <= 10){
//					motor_dir = 0;
//					motor_angle = max_right;
//				}
//				else if (IR_Dist_1 > 25 && IR_Dist_2 > 25){
//					motor_dir = 1;
//					motor_angle = center;
//				}
//			}
//
//			//	Robot is trying to go left
//			//	Upon reaching an obstacle, reverse in the other direction
//			else if (target_angle > 0){
//				if (IR_Dist_1 <= 10 || IR_Dist_2 <= 10){
//					motor_dir = 0;
//					motor_angle = max_right;
//				}
//				else if (IR_Dist_1 > 25 && IR_Dist_2 > 25){
//					motor_dir = 1;
//					motor_angle = max_left;
//				}
//			}
//
//			//	Robot is trying to go right
//			//	Upon reaching an obstacle, reverse in the other direction
//			else{
//				if (IR_Dist_1 <= 10 || IR_Dist_2 <= 10){
//					motor_dir = 0;
//					motor_angle = max_left;
//				}
//				else if (IR_Dist_1 > 25 && IR_Dist_2 > 25){
//					motor_dir = 1;
//					motor_angle = max_right;
//				}
//			}

			osDelay(1);
		}

	}
	/* USER CODE END IRFunc */
}

/* USER CODE BEGIN Header_turningFunc */
/**
 * @brief Function implementing the turningTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_turningFunc */
void turningFunc(void const *argument) {
	/* USER CODE BEGIN turningFunc */
	/* Infinite loop */
	//	Assume left is positive and right is negative
	for (;;) {
		if (currently_running == 2) {
//		  if (target_angle > 0){
//			  motor_angle = max_left;
//		  }
//		  else if (target_angle == 0){
//			  motor_angle = center;
//		  }
//		  else{
//			  motor_angle = max_right;
//		  }
			if (target_angle > 0 && yawG >= target_angle) {
				//pwm_val_L = 0;
				//pwm_val_R = 0;
				motor_angle = center;
				currently_running = 0;
				target_angle = 0; // Reset to moving forward
				motor_dir = 1;
			}
			if (target_angle < 0 && yawG <= target_angle) {
				//pwm_val_L = 0;
				//pwm_val_R = 0;
				motor_angle = center;
				currently_running = 0;
				target_angle = 0; // Reset to moving forward
				motor_dir = 1;
			}
		}
		osDelay(1);
	}
	/* USER CODE END turningFunc */
}

/* USER CODE BEGIN Header_ultrasonic */
/**
 * @brief Function implementing the ultrasonciTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ultrasonic */
void ultrasonic(void const *argument) {
	/* USER CODE BEGIN ultrasonic */
	uint8_t usValueBuffer[20] = { 0 };
	/* Infinite loop */
	for (;;) {
		if (currently_running == 2) {
			Ultrasonic_Read();
			sprintf(usValueBuffer, "dist: %4d", (int) Distance);
//		  HAL_UART_Transmit(&huart3, (uint8_t*) usValueBuffer, strlen(usValueBuffer), 0xFFFF);
			OLED_ShowString(10, 50, usValueBuffer);
		}
		osDelay(1);
	}
	/* USER CODE END ultrasonic */
}

/* USER CODE BEGIN Header_obstacleTask */
/**
 * @brief Function implementing the task2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_obstacleTask */
void obstacleTask(void const *argument) {
	/* USER CODE BEGIN obstacleTask */
	/* Infinite loop */

	//	Flags to check for distance when moving along second obstacle
	int checkLeftIR = 0;
	int checkRightIR = 0;

	int oldIndex = 0;
	int xOldDist = 0;
	int yOldDist = 0;
	int xDistTravelled = 0;
	int yDistTravelled = -1;
	int requiredX = 0;
	int finalIndex = 0;
	int tempX = 0;

	int temp = -1;

	uint8_t xDistTravelledStr[20];
	uint8_t yDistTravelledStr[20];

	uint8_t snap1[10] = "snap1";
	uint8_t snap2[10] = "snap2";

	int sent1 = 0;
	int sent2 = 0;

	for (;;) {
		if (currently_running == 2) {
			if (manualReset)
			{
				checkLeftIR = 0;
				checkRightIR = 0;
				oldIndex = 0;
				xOldDist = 0;
				yOldDist = 0;
				xDistTravelled = 0;
				yDistTravelled = -1;
				requiredX = 0;
				finalIndex = 0;
				tempX = 0;
				temp = -1;
				sent1 = 0;
				sent2 = 0;
				manualReset = 0;
			}
			//	Makes the car stop at a specified distance from the ultrasonic sensor
			if (stopDist != 0 && motor_dir == 1) {
				if ((int) Distance <= stopDist) {
					stopAll();
//					HAL_UART_Transmit(&huart3, (uint8_t*) success, 1, 0xFFFF);
					osDelay(700);
//					Ultrasonic_Read();
					//	Move back a bit if distance is too close
					if ((int) Distance < stopDist) {
						int diff = (stopDist - (int) Distance) * 10;
						sprintf(aRxBuffer, "mb%d     ", diff);
						stopDist = 0;
						checkBits();
//						HAL_UART_Transmit(&huart3, (uint8_t*) aRxBuffer,
//								strlen(aRxBuffer), 0xFFFF);
						sprintf(aRxBuffer, "");
					}
					stopDist = 0;
				}
			}
			//	If moving backwards, don't need to check for sensor distance
			else if (stopDist != 0 && motor_dir == 0) {
				stopDist = 0;
			}

			if (startTask2) {
				yDistTravelled = 0;
				yOldDist = distA;
				sprintf(aRxBuffer, "mf10000");
				checkBits();
				sprintf(aRxBuffer, "");
				startTask2 = 0;
			}

			if (stopDist == 0 && yDistTravelled == 0) {
				yDistTravelled = distA - yOldDist;
				sprintf(yDistTravelledStr, "Up %d\n\r", yDistTravelled);
//				HAL_UART_Transmit(&huart3, (uint8_t*) yDistTravelledStr,
//						strlen(yDistTravelledStr), 0xFFFF);
				currentMov = 1;

			}

			//	Car stops at first obstacle, call image rec
			if (currentMov == 1 && !sent1)
			{
				//	Call image rec
				HAL_UART_Transmit(&huart3, (uint8_t*) snap1,strlen(snap1), 0xFFFF);
				sent1 = 1;
			}

			//	Car goes past first obstacle, ready to move straight to second obstacle
			if (currentMov == 2 && !secondStraight)
			{
				osDelay(700);
				sprintf(aRxBuffer,"2");
				checkBits();
				sprintf(aRxBuffer,"");
			}

			//	Car stops at second obstacle, call image rec
			if (currentMov == 3 && !sent2)
			{
				//	Call image rec
				HAL_UART_Transmit(&huart3, (uint8_t*) snap2,strlen(snap2), 0xFFFF);
				sent2 = 1;
			}

			//	Car goes past second obstacle, ready to move straight to go back
			if (currentMov == 4 && !returnCarPark)
			{
				osDelay(700);
				sprintf(aRxBuffer,"3");
				checkBits();
				sprintf(aRxBuffer,"");
			}

			//	Car does the final turns to align itself to carpark
			if (currentMov == 5 && !finalTurns && dist_to_travel == 0)
			{
				osDelay(700);
				sprintf(aRxBuffer,"4");
				checkBits();
				sprintf(aRxBuffer,"");
			}
			if (secondStraight) {
				temp = 0;
				yOldDist = distA;
				sprintf(aRxBuffer, "mf10000");
				checkBits();
				sprintf(aRxBuffer, "");
				secondStraight = 0;
				currentMov = 3;
			}

			if (stopDist == 0 && temp == 0) {
				temp = distA - yOldDist;
				yDistTravelled += temp;
				sprintf(yDistTravelledStr, "Up %d, total %d\n\r", temp,
						yDistTravelled);
//				HAL_UART_Transmit(&huart3, (uint8_t*) yDistTravelledStr,
//						strlen(yDistTravelledStr), 0xFFFF);

			}

			//	Obstacle 2 shows left arrow
			if (scenarioL_2) {
				if (target_angle == 0 && dist_to_travel == 0 && index_2 < 6) {
					osDelay(400);
					sprintf(aRxBuffer, L_Instructions_2[index_2]);
					if (index_2 == 2 || index_2 == 4) {
						checkRightIR = 1;
					}

					//	Take note of distance travelled to use for going back to carpark
					if (index_2 == 4 && xOldDist == 0) {
						xOldDist = distA;
					}
					if (index_2 == 5 && xDistTravelled == 0) {
						xDistTravelled = distA - xOldDist;
					}
					sprintf(xDistTravelledStr, "Moved %d\n\r", xDistTravelled);
//					HAL_UART_Transmit(&huart3, (uint8_t*) distTravelledStr, strlen(distTravelledStr), 0xFFFF);

					index_2++;
					checkBits();
					sprintf(aRxBuffer, "");
				} else if (index_2 == 6 && target_angle == 0
						&& dist_to_travel == 0) {
					sprintf(aRxBuffer, "");
					stopAll();
					xOldDist = 0;
					currentMov = 4;
//					xDistTravelled = 0;
				}
			}
			if (scenarioR_2) {
				if (target_angle == 0 && dist_to_travel == 0 && index_2 < 6) {
					osDelay(400);
					sprintf(aRxBuffer, R_Instructions_2[index_2]);
					if (index_2 == 2 || index_2 == 4) {
						checkLeftIR = 1;
					}

					//	Take note of distance travelled to use for going back to carpark
					if (index_2 == 4 && xOldDist == 0) {
						xOldDist = distA;
					}
					if (index_2 == 5 && xDistTravelled == 0) {
						xDistTravelled = distA - xOldDist;
					}
					sprintf(xDistTravelledStr, "Moved %d\n\r", xDistTravelled);
//					HAL_UART_Transmit(&huart3, (uint8_t*) distTravelledStr, strlen(distTravelledStr), 0xFFFF);

					index_2++;
					checkBits();
					sprintf(aRxBuffer, "");
				} else if (index_2 == 6 && target_angle == 0
						&& dist_to_travel == 0) {
					sprintf(aRxBuffer, "");
					stopAll();
					xOldDist = 0;
					currentMov = 4;
//					xDistTravelled = 0;
				}
			}

			if (returnCarPark) {
				yDistTravelled += firstTurnLength;
				temp += firstTurnLength;
				temp+=170;
				sprintf(aRxBuffer, "mf%d", temp);
				checkBits();
				sprintf(aRxBuffer, "");
				returnCarPark = 0;
				currentMov = 5;
			}

			if (finalTurns) {
				if (!requiredX) {
					requiredX = xDistTravelled / 2;
					requiredX += turn90xDisplacement;
				}
				//	Car located left of carpark
				if (carLocation == 0 && requiredX != 0) {
					if (finalIndex == 0) {
						osDelay(500);
						sprintf(aRxBuffer, "tr90");
						requiredX -= turn90xDisplacement;
						requiredX-=50;
						checkBits();
						sprintf(aRxBuffer, "");
						finalIndex++;
					}
					if (finalIndex == 1 && target_angle == 0) {
						osDelay(500);
						if (requiredX > turn90xDisplacement) {
							tempX = requiredX - turn90xDisplacement;
							requiredX -= tempX;
							sprintf(aRxBuffer, "mf%d", tempX);
							checkBits();
							sprintf(aRxBuffer, "");
						}
						else
						{
							tempX = turn90xDisplacement - requiredX;
							requiredX = turn90xDisplacement;
							sprintf(aRxBuffer, "mb%d", tempX);
							checkBits();
							sprintf(aRxBuffer, "");
						}
						finalIndex++;
					}
					if (finalIndex == 2 && dist_to_travel == 0)
					{
						osDelay(500);
						sprintf(aRxBuffer, "tl87");
						checkBits();
						sprintf(aRxBuffer, "");
						finalIndex++;
					}
					if (finalIndex == 3 && target_angle == 0
							&& dist_to_travel == 0) {
						currentMov = 6;
						finalTurns = 0;
					}
				}

				//	Car located right of carpark
				if (carLocation == 1 && requiredX != 0) {
					if (finalIndex == 0) {
						osDelay(500);
						sprintf(aRxBuffer, "tl85");
						requiredX -= turn90xDisplacement;
						checkBits();
						sprintf(aRxBuffer, "");
						finalIndex++;
					}
					if (finalIndex == 1 && target_angle == 0) {
						osDelay(500);
						if (requiredX > turn90xDisplacement) {
							tempX = requiredX - turn90xDisplacement;
							requiredX -= tempX;
							sprintf(aRxBuffer, "mf%d", tempX);
							checkBits();
							sprintf(aRxBuffer, "");
						}
						else
						{
							tempX = turn90xDisplacement - requiredX;
//							tempX+=30;
							requiredX = turn90xDisplacement;
							sprintf(aRxBuffer, "mb%d", tempX);
							checkBits();
							sprintf(aRxBuffer, "");
						}
						finalIndex++;
					}
					if (finalIndex == 2 && dist_to_travel == 0)
					{
						osDelay(500);
						sprintf(aRxBuffer, "tr90");
						checkBits();
						sprintf(aRxBuffer, "");
						finalIndex++;
					}
					if (finalIndex == 3 && target_angle == 0
							&& dist_to_travel == 0) {
						currentMov = 6;
						finalTurns = 0;
					}
				}
			}

			if (currentMov == 6)
			{
				osDelay(700);
				currentMov = 7;
				stopDist = 10;
				sprintf(aRxBuffer,"mf10000");
				checkBits();
				sprintf(aRxBuffer,"");
			}

			//	Everything done (hopefully)
			if (currentMov == 7 && stopDist == 0)
			{
				HAL_UART_Transmit(&huart3, (uint8_t*) success, 1, 0xFFFF);
				currentMov = 8;
			}
			//	Checking right IR sensor to see when the robot moves past the obstacle
			if (checkRightIR) {
				if (IR_Dist_2 > 50) {
					oldIndex = index_2;
					stopAll();
					sprintf(aRxBuffer, "");
					checkRightIR = 0;
					if (oldIndex == 3 || oldIndex == 5) {
						index_2 = oldIndex;
						scenarioL_2 = 1;
					}
				}
			}
			if (checkLeftIR) {
				if (IR_Dist_1 > 50) {
					oldIndex = index_2;
					stopAll();
					sprintf(aRxBuffer, "");
					checkLeftIR = 0;
					if (oldIndex == 3 || oldIndex == 5) {
						index_2 = oldIndex;
						scenarioR_2 = 1;
					}

				}
			}
		}
	}
	/* USER CODE END obstacleTask */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
