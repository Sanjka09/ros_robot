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
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "mainpp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CAN1_DEVICE_NUM     4
#define FIRST_GROUP_ID      0x200
#define MOTOR_SPEED_MAX     16384
#define CAN_DATA_SIZE       8
#define CAN1_RX_ID_START    0x201
#define MOTOR_ID            4
#define spi_enable      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define spi_disable     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct
{
	float KP;
	float KI;
	float KD;
	int error;
	int I_error;
	int D_error;
	int lastError;

} PID_Variables;

typedef struct
{
	int16_t setpoint;
	int16_t Out;
	uint16_t ID;
	int16_t en_speed;
} M3508_Variables;

typedef struct
{
	int LY;
	int LX;
	int RY;
	int RX;
	double X;
	double Y;
}joystick_variables;

typedef struct
{
	int target_pos;
	int32_t my_pos;
	float ControlSignal;
	int PWM;
}DC_Motor;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId WheelControlHandle;
osThreadId JoystickHandle;
osThreadId up_down_controlHandle;
osThreadId Target_posHandle;
osThreadId ColorHandle;
/* USER CODE BEGIN PV */


double pi=3.14159265359;
double ML;
double MR;
double speed;
double degree_a;

float freq=0;
float dif=0;
float cTime=0;
float pTime=0;
float dTime=0;


int vel_up=0;
int motor_dir=0;
int is_first=0;
int ic1=0;
int ic2=0;
int red=0;
int blue=0;
int yellow=0;
int flag=0;
int BLDC = 0;

uint8_t HC_PS2_RX[9];
uint8_t HC_PS2_TX[9]={0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};
uint8_t RxData[1];


uint8_t             data[8];
uint32_t            pTxMailbox;
uint8_t 			rxData[8];
PID_Variables M1_pid;
PID_Variables M2_pid;
PID_Variables M3_pid;
PID_Variables M4_pid;
PID_Variables DC_pid;


M3508_Variables M1;
M3508_Variables M2;
M3508_Variables M3;
M3508_Variables M4;

DC_Motor Motor;
joystick_variables PS2;

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
void Wheel_task(void const * argument);
void task2_joystick(void const * argument);
void DC_motor(void const * argument);
void Colorcheck(void const * argument);
void StartTask05(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void can_transmit(CAN_HandleTypeDef* hcan1, uint16_t id, int16_t msg1, int16_t msg2, int16_t msg3, int16_t msg4);
void calculatePID(void);
void motorspeed(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

//  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);
  //HAL_UART_Receive_IT(&huart4, RxData, 1);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);


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
  /* definition and creation of WheelControl */
  osThreadDef(WheelControl, Wheel_task, osPriorityNormal, 0, 128);
  WheelControlHandle = osThreadCreate(osThread(WheelControl), NULL);

  /* definition and creation of Joystick */
  osThreadDef(Joystick, task2_joystick, osPriorityNormal, 0, 2048);
  JoystickHandle = osThreadCreate(osThread(Joystick), NULL);

  /* definition and creation of up_down_control */
  osThreadDef(up_down_control, DC_motor, osPriorityNormal, 0, 512);
  up_down_controlHandle = osThreadCreate(osThread(up_down_control), NULL);

  /* definition and creation of Target_pos */
  osThreadDef(Target_pos, Colorcheck, osPriorityNormal, 0, 512);
  Target_posHandle = osThreadCreate(osThread(Target_pos), NULL);

  /* definition and creation of Color */
  osThreadDef(Color, StartTask05, osPriorityNormal, 0, 128);
  ColorHandle = osThreadCreate(osThread(Color), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  M1.ID=0x201;
  M2.ID=0X202;
  M3.ID=0x203;
  M4.ID=0x204;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x205;
  sFilterConfig.FilterIdLow = 0x200;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
    	Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    	Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    	Error_Handler();
  }


  /* USER CODE END CAN1_Init 2 */

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
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 640-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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
  htim8.Init.Prescaler = 2650-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100-1;
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
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, S2_Pin|S3_Pin|IN4_Pin|IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : UP1_Pin DOWN2_Pin DOWN1_Pin BALL1_Pin */
  GPIO_InitStruct.Pin = UP1_Pin|DOWN2_Pin|DOWN1_Pin|BALL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BALL2_Pin */
  GPIO_InitStruct.Pin = BALL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BALL2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S2_Pin S3_Pin IN4_Pin IN3_Pin */
  GPIO_InitStruct.Pin = S2_Pin|S3_Pin|IN4_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UP2_Pin IR_Pin */
  GPIO_InitStruct.Pin = UP2_Pin|IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin IN1_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance==TIM5){
//		Motor.my_pos = ((int32_t)__HAL_TIM_GET_COUNTER(htim)) / 100;
//	}
//	if(htim-> Instance == TIM3){
//		  if(is_first == 0){
//			  ic1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//			  is_first = 1;
//		  }else
//		  {
//			  ic2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//			  if(ic2>ic1)
//			  {
//				  dif = ic2 - ic1;
//			  }
//			  else if(ic1>ic2){
//				  dif = (0xffffffff-ic1) + ic2;
//			  }
//			  freq = 1000/dif;
//			  if(freq>50 && freq<95){
//				  blue = 1;
//			  }else
//				  blue =0;
////			  HAL_Delay(500);
//			  __HAL_TIM_SET_COUNTER(htim, 0);
//			  is_first = 0;
//		  }
//	  }
//}

void MotorUp(void)
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

void MotorDown(void)
{

	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
}

void MotorStop(void)
{
	TIM8 -> CCR1 = 99;
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
}

void calculatePID()
{
	cTime = HAL_GetTick();
	Motor.my_pos = ((int32_t)TIM5->CNT)/100;
	dTime = (cTime - pTime)/10000;
	DC_pid.error = Motor.target_pos - Motor.my_pos;
	DC_pid.I_error += (DC_pid.error * dTime);
	DC_pid.D_error = (DC_pid.error - DC_pid.lastError)/dTime;
	Motor.ControlSignal = (DC_pid.KP*DC_pid.error) + (DC_pid.I_error*DC_pid.KI) + (DC_pid.D_error*DC_pid.KD);
	HAL_Delay(1);
	pTime = cTime;
	DC_pid.lastError = DC_pid.error;
}

void motorspeed(){
	if(Motor.ControlSignal<0)
		motor_dir = -1;
	else if(Motor.ControlSignal>0)
		motor_dir = 1;
	else
		motor_dir = 0;
	Motor.PWM = (int)fabs(Motor.ControlSignal);
	if(Motor.PWM > 300)
		TIM8 -> CCR1 = 99;
//	TIM8 -> CCR2 = 50;
	if(Motor.PWM < 300 && DC_pid.error != 0){
		TIM8 -> CCR1 = 40;
	}
	if(motor_dir == 1){
		MotorUp();
	}else if(motor_dir == (-1)){
		MotorDown();
	}else{
		MotorStop();
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//	uint8_t rxData[8];
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData);
	if(rxHeader.StdId == 0x201){
		M1.en_speed = rxData[2] << 8;
		M1.en_speed = M1.en_speed + rxData[3];
		M1.en_speed = M1.en_speed * 1.5;
	}
	if(rxHeader.StdId == 0x202){
		M2.en_speed = rxData[2] << 8;
		M2.en_speed = M2.en_speed + rxData[3];
		M2.en_speed = M2.en_speed * 1.5;
	}
	if(rxHeader.StdId == 0x203){
		M3.en_speed = rxData[2] << 8;
		M3.en_speed = M3.en_speed + rxData[3];
		M3.en_speed = M3.en_speed * 1.5;
	}
	if(rxHeader.StdId == 0x204){
		M4.en_speed = rxData[2] << 8;
		M4.en_speed = M4.en_speed + rxData[3];
		M4.en_speed = M4.en_speed * 1.5;
		vel_up = 1;
	}
}
void can_transmit(CAN_HandleTypeDef* hcan, uint16_t id, int16_t msg1, int16_t msg2, int16_t msg3, int16_t msg4){
    CAN_TxHeaderTypeDef tx_header;
//    uint8_t             data[8];
//    uint32_t            pTxMailbox;

    tx_header.StdId = id;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = CAN_DATA_SIZE;
    tx_header.TransmitGlobalTime = DISABLE;
    data[0] = msg1 >> 8;
    data[1] = msg1;
    data[2] = msg2 >> 8;
    data[3] = msg2;
    data[4] = msg3 >> 8;
    data[5] = msg3;
    data[6] = msg4 >> 8;
    data[7] = msg4;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &pTxMailbox) == HAL_OK){
        while (HAL_CAN_IsTxMessagePending(hcan, pTxMailbox));
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Wheel_task */
/**
  * @brief  Function implementing the WheelControl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Wheel_task */
void Wheel_task(void const * argument)
{
  /* USER CODE BEGIN 5 */
	M1_pid.KP = 1.5;
	M1_pid.KI = 0.0005;
	M1_pid.KD = 0.01;

	M2_pid.KP = 1.5;
	M2_pid.KI = 0.0005;
	M2_pid.KD = 0.01;

	M3_pid.KP = 1.5;
	M3_pid.KI = 0.0005;
	M3_pid.KD = 0.01;

	M4_pid.KP = 1.5;
	M4_pid.KI = 0.0005;
	M4_pid.KD = 0.01;
  /* Infinite loop */
  for(;;)
  {

  if(vel_up==1){
	  vel_up=0;
		  M1_pid.error=M1.setpoint-M1.en_speed;
		  M1_pid.I_error+=M1_pid.error;
		  M1_pid.D_error=M1_pid.lastError-M1_pid.error;
		  M1_pid.lastError=M1_pid.error;
		  M1.Out=M1_pid.KP*M1_pid.error+M1_pid.KI*M1_pid.I_error+M1_pid.KD*M1_pid.D_error;

		  M2_pid.error=M2.setpoint-M2.en_speed;
		  M2_pid.I_error+=M2_pid.error;
		  M2_pid.D_error=M2_pid.lastError-M2_pid.error;
		  M2_pid.lastError=M2_pid.error;
		  M2.Out=M2_pid.KP*M2_pid.error+M2_pid.KI*M2_pid.I_error+M2_pid.KD*M2_pid.D_error;


		  M3_pid.error=M3.setpoint-M3.en_speed;
		  M3_pid.I_error+=M3_pid.error;
		  M3_pid.D_error=M3_pid.lastError-M3_pid.error;
		  M3_pid.lastError=M3_pid.error;
		  M3.Out=M3_pid.KP*M3_pid.error+M3_pid.KI*M3_pid.I_error+M3_pid.KD*M3_pid.D_error;


		  M4_pid.error=M4.setpoint-M4.en_speed;
		  M4_pid.I_error+=M4_pid.error;
		  M4_pid.D_error=M4_pid.lastError-M4_pid.error;
		  M4_pid.lastError=M4_pid.error;
		  M4.Out=M4_pid.KP*M4_pid.error+M4_pid.KI*M4_pid.I_error+M4_pid.KD*M4_pid.D_error;

		  can_transmit(&hcan1, FIRST_GROUP_ID, M1.Out, M2.Out, M3.Out, M4.Out);
 }

    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task2_joystick */
/**
* @brief Function implementing the Joystick thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task2_joystick */
void task2_joystick(void const * argument)
{
  /* USER CODE BEGIN task2_joystick */

  /* Infinite loop */
  for(;;)
  {
	  spi_enable;
	  HAL_SPI_TransmitReceive(&hspi2, HC_PS2_TX, HC_PS2_RX, 9, 10);
	  spi_disable;
	  PS2.LY=-(HC_PS2_RX[8]-127);
	  PS2.LX=(HC_PS2_RX[7]-127);
	  PS2.RY=HC_PS2_RX[6]-128;
	  PS2.RX=HC_PS2_RX[5]-128;
	  PS2.X=PS2.LX/(float)128;
	  PS2.Y=PS2.LY/(float)128;
	  speed=sqrt(PS2.X*PS2.X+PS2.Y*PS2.Y);
	  degree_a=atan2(PS2.Y,PS2.X);
	  ML=sin(degree_a-pi/4)*speed*MOTOR_SPEED_MAX;
	  MR=cos(degree_a-pi/4)*speed*MOTOR_SPEED_MAX;
	  if(HC_PS2_RX[4]==251){
		  BLDC = 1;
	  }else if(HC_PS2_RX[4]==254){
		  BLDC = 2;
	  }else if(HC_PS2_RX[4]==247){
		  yellow=1;
		  Motor.target_pos = 1800;
	  }else if(HC_PS2_RX[4]==253){
		  yellow=2;
		  Motor.target_pos = 0;
	  }
	  if((abs(PS2.LY) > 5 || abs(PS2.LX)>5 )&& abs(PS2.RX)<=5){
		  if(ML>12000 || MR>12000){
			  if(ML > MR){
				  MR=MR/ML*12000;
				  ML=12000;

			  }
			  else{
				  ML=(ML/MR)*12000;
				  MR=12000;
			  }
		  }
		  else if(ML<-12000 || MR<-12000){
			  if(ML < MR){
				  MR=-MR/ML*12000;
				  ML=-12000;

			  }
			  else{
				  ML=-ML/MR*12000;
				  MR=-12000;
			  }
		  }
		  else if(ML>12000 || MR<-12000){
			  if(ML > -MR){
				  MR=MR/ML*12000;
				  ML=12000;

			  }
			  else{
				  ML=-ML/MR*12000;
				  MR=-12000;
			  }
		  }
		  else if(MR>12000 || ML<-12000){
			  if(MR > -ML){
				  ML=ML/MR*12000;
				  MR=12000;

			  }
			  else{
				  MR=-MR/ML*12000;
				  ML=-12000;
			  }
		  }
		  M1.setpoint=-MR;
		  M2.setpoint=ML;
		  M4.setpoint=-ML;
		  M3.setpoint=MR;
	  }
	  else if(abs(PS2.LY)<=5 && abs(PS2.LX)<=5 && abs(PS2.RX)>5){
		  M1.setpoint=-MOTOR_SPEED_MAX*PS2.RX/500;
		  M2.setpoint=-MOTOR_SPEED_MAX*PS2.RX/500;
		  M3.setpoint=-MOTOR_SPEED_MAX*PS2.RX/500;
		  M4.setpoint=-MOTOR_SPEED_MAX*PS2.RX/500;
	  }
	  else if((abs(PS2.LY)>5 || abs(PS2.LX)>5) && abs(PS2.RX)>5){
		  PS2.X=PS2.LX/(float)128;
		  PS2.Y=PS2.LY/(float)128;
		  speed=sqrt(PS2.X*PS2.X+PS2.Y*PS2.Y);
//			  if(speed>=1){
////				  speed=sign(speed);
//				  speed=1;
//			  }
//			  if(speed<=-1){
//				  speed=-1;
//			  }
		  degree_a=atan2(PS2.Y,PS2.X);
		  ML=sin(degree_a-pi/4)*speed*MOTOR_SPEED_MAX;
		  MR=cos(degree_a-pi/4)*speed*MOTOR_SPEED_MAX;
		  if(ML>12000 || MR>12000){
			  if(ML > MR){
				  MR=MR/ML*12000;
				  ML=12000;

			  }
			  else{
				  ML=ML/MR*12000;
				  MR=12000;
			  }
		  }
		  else if(ML<-12000 || MR<-12000){
			  if(ML < MR){
				  MR=-MR/ML*12000;
				  ML=-12000;

			  }
			  else{
				  ML=-ML/MR*12000;
				  MR=-12000;
			  }
		  }
		  else if(ML>12000 || MR<-12000){
			  if(ML > -MR){
				  MR=MR/ML*12000;
				  ML=12000;

			  }
			  else{
				  ML=-ML/MR*12000;
				  MR=-12000;
			  }
		  }
		  else if(MR>12000 || ML<-12000){
			  if(MR > -ML){
				  ML=ML/MR*12000;
				  MR=12000;

			  }
			  else{
				  MR=-MR/ML*12000;
				  ML=-12000;
			  }
		  }
		  M1.setpoint=-MR;
		  M2.setpoint=ML;
		  M4.setpoint=-ML;
		  M3.setpoint=MR;
		  M1.setpoint=M1.setpoint-(MOTOR_SPEED_MAX*PS2.RX/800);
		  M2.setpoint=M2.setpoint-(MOTOR_SPEED_MAX*PS2.RX/800);
		  M3.setpoint=M3.setpoint-(MOTOR_SPEED_MAX*PS2.RX/800);
		  M4.setpoint=M4.setpoint-(MOTOR_SPEED_MAX*PS2.RX/800);
	  }
	  else{
		  M1.setpoint=0;
		  M2.setpoint=0;
		  M3.setpoint=0;
		  M4.setpoint=0;
	  }
      osDelay(10);
  }
  /* USER CODE END task2_joystick */
}

/* USER CODE BEGIN Header_DC_motor */
/**
* @brief Function implementing the up_down_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DC_motor */
void DC_motor(void const * argument)
{
  /* USER CODE BEGIN DC_motor */
	DC_pid.error = 0;
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_SET);
	TIM4->CCR1=920;
	TIM4->CCR2=920;
//	TIM8 -> CCR1 = 99;
  /* Infinite loop */
  for(;;)
  {
		if(BLDC==1 || flag==1){
			TIM4->CCR1 = 930;
			TIM4->CCR2 = 909;
			flag = 1;
			//HAL_Delay(500);
			while((HAL_GPIO_ReadPin(BALL1_GPIO_Port, BALL1_Pin)==0) || (HAL_GPIO_ReadPin(BALL2_GPIO_Port, BALL2_Pin)==0)){
				TIM4->CCR1 = 920;
				TIM4->CCR2 = 920;
//				HAL_Delay(2000);
				flag = 0;
				BLDC = 0;
				blue = 1;
				Motor.target_pos = 890;
				break;
//				if(blue==1 || red==1){
//					Motor.target_pos = 1800;
//					break;
//				}else{
//					Motor.target_pos = 890;
//					break;
//				}
			}
		}
		else if(BLDC==2 && flag==0){
			TIM4->CCR1 = 915;
			TIM4->CCR2 = 925;
			//HAL_Delay(200);
			while((HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin)) == 1){
				TIM4->CCR1 = 920;
				TIM4->CCR2 = 920;
				HAL_Delay(2000);
				BLDC = 0;
				yellow=2;
				Motor.target_pos = 0;
				break;
			}
		}
    osDelay(50);
  }
  /* USER CODE END DC_motor */
}

/* USER CODE BEGIN Header_Colorcheck */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Colorcheck */
void Colorcheck(void const * argument)
{
  /* USER CODE BEGIN Colorcheck */
	Motor.my_pos = TIM5->CNT=0;
	DC_pid.KP = 0.7;
	DC_pid.KI = 0.0005;
	DC_pid.KD = 0.00003;
	Motor.target_pos=0;
  /* Infinite loop */
  for(;;)
  {
	 if(blue == 1 && Motor.target_pos == 890){//taget pos = 890
		 calculatePID();
		 motorspeed();
		 Motor.target_pos = 890;
		 while(DC_pid.error < 5 ){
			 MotorStop();
			 TIM5->CNT=89000;
			 Motor.my_pos=890;
//			 Motor.target_pos = 0;
//			 yellow=2;
			 blue = 0;
			 break;
		}
	 }else if(yellow==2 && Motor.target_pos == 0){//target pos = 0
		 while(((HAL_GPIO_ReadPin(DOWN1_GPIO_Port, DOWN1_Pin))==0 || (HAL_GPIO_ReadPin(DOWN2_GPIO_Port, DOWN2_Pin))==0) && DC_pid.error < 100){
			 MotorStop();
//			 Motor.target_pos=0;
			 TIM5->CNT=0;
			 Motor.my_pos=0;
			 yellow=0;
			 break;
		 }
		 	 Motor.target_pos= 0;
			 calculatePID();
			 motorspeed();
	}else if(yellow==1 && Motor.target_pos == 1800){//target pos = 1800
		 while(((HAL_GPIO_ReadPin(UP1_GPIO_Port, UP1_Pin))==0 || (HAL_GPIO_ReadPin(UP2_GPIO_Port, UP2_Pin))==0) && DC_pid.error < 100){
			 MotorStop();
			 Motor.my_pos = 1800;
			 Motor.target_pos = 0;
			 TIM5->CNT = 180000;
			 yellow=0;
			 break;
		 }
		 	 Motor.target_pos = 1800;
			 calculatePID();
			 motorspeed();
	}else{
		 MotorStop();
		 yellow = 0;
	 }
    osDelay(5);
  }
  /* USER CODE END Colorcheck */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the BLDC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
	setup();
  /* Infinite loop */
  for(;;)
  {
	  loop();
    osDelay(10);
  }
  /* USER CODE END StartTask05 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
