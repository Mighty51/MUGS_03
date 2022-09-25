/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

//#include "I2C_Dev.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */
CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHANNEL_1_LED_OK HAL_GPIO_WritePin(ch1_1_GPIO_Port, ch1_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(ch1_2_GPIO_Port, ch1_2_Pin, GPIO_PIN_SET);
#define CHANNEL_1_LED_OFF HAL_GPIO_WritePin(ch1_1_GPIO_Port, ch1_1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(ch1_2_GPIO_Port, ch1_2_Pin, GPIO_PIN_RESET);
#define CHANNEL_2_LED_OK HAL_GPIO_WritePin(ch2_1_GPIO_Port, ch2_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(ch2_2_GPIO_Port, ch2_2_Pin, GPIO_PIN_SET);
#define CHANNEL_2_LED_OFF HAL_GPIO_WritePin(ch2_1_GPIO_Port, ch2_1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(ch2_2_GPIO_Port, ch2_2_Pin, GPIO_PIN_RESET);
#define CHANNEL_3_LED_OK HAL_GPIO_WritePin(ch3_1_GPIO_Port, ch3_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(ch3_2_GPIO_Port, ch3_2_Pin, GPIO_PIN_SET);
#define CHANNEL_3_LED_OFF HAL_GPIO_WritePin(ch3_1_GPIO_Port, ch3_1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(ch3_2_GPIO_Port, ch3_2_Pin, GPIO_PIN_RESET);
#define CHANNEL_4_LED_OK HAL_GPIO_WritePin(ch4_1_GPIO_Port, ch4_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(ch4_2_GPIO_Port, ch4_2_Pin, GPIO_PIN_SET);
#define CHANNEL_4_LED_OFF HAL_GPIO_WritePin(ch4_1_GPIO_Port, ch4_1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(ch4_2_GPIO_Port, ch4_2_Pin, GPIO_PIN_RESET);
#define STATUS_LED_GREEN HAL_GPIO_WritePin(on1_GPIO_Port, on1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(on2_GPIO_Port, on2_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(f1_GPIO_Port, f1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(f2_GPIO_Port, f2_Pin,GPIO_PIN_RESET);
#define STATUS_LED_OFF HAL_GPIO_WritePin(on1_GPIO_Port, on1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(on2_GPIO_Port, on2_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(f1_GPIO_Port, f1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(f2_GPIO_Port, f2_Pin,GPIO_PIN_RESET);										
#define STATUS_LED_RED HAL_GPIO_WritePin(on1_GPIO_Port, on1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(on2_GPIO_Port, on2_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(f1_GPIO_Port, f1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(f2_GPIO_Port, f2_Pin,GPIO_PIN_SET);
#define STATUS_LED_YELLOW HAL_GPIO_WritePin(on1_GPIO_Port, on1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(on2_GPIO_Port, on2_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(f1_GPIO_Port, f1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(f2_GPIO_Port, f2_Pin,GPIO_PIN_SET);

#define CHANNEL_1_POWER_ON HAL_GPIO_WritePin(KeyOn1_GPIO_Port,KeyOn1_Pin,GPIO_PIN_SET);
#define CHANNEL_1_POWER_OFF HAL_GPIO_WritePin(KeyOn1_GPIO_Port,KeyOn1_Pin,GPIO_PIN_RESET);
#define CHANNEL_2_POWER_ON HAL_GPIO_WritePin(KeyOn2_GPIO_Port,KeyOn2_Pin,GPIO_PIN_SET);
#define CHANNEL_2_POWER_OFF HAL_GPIO_WritePin(KeyOn2_GPIO_Port,KeyOn2_Pin,GPIO_PIN_RESET);
#define CHANNEL_3_POWER_ON HAL_GPIO_WritePin(KeyOn3_GPIO_Port,KeyOn3_Pin,GPIO_PIN_SET);
#define CHANNEL_3_POWER_OFF HAL_GPIO_WritePin(KeyOn3_GPIO_Port,KeyOn3_Pin,GPIO_PIN_RESET);
#define CHANNEL_4_POWER_ON HAL_GPIO_WritePin(KeyOn4_GPIO_Port,KeyOn4_Pin,GPIO_PIN_SET);
#define CHANNEL_4_POWER_OFF HAL_GPIO_WritePin(KeyOn4_GPIO_Port,KeyOn4_Pin,GPIO_PIN_RESET);

#define TEST_ZapretSmazki HAL_GPIO_ReadPin(ZS_GPIO_Port,ZS_Pin)
#define TEST_BAK HAL_GPIO_ReadPin(DB_GPIO_Port,DB_Pin)
#define ZapretSmazki_ON GPIO_PIN_SET
#define ZapretSmazki_OFF GPIO_PIN_RESET
#define BAK_ON GPIO_PIN_SET
#define BAK_OFF GPIO_PIN_RESET

#define SENSOR_BUS hi2c2

#define I2C_TIMEOUT 10
#define GYRO_I2C_ADRESS 0xD0
#define GYRO_I2C_ID_Who_Am_I 0x0F
#define GYRO_I2C_ID_OUT_TEMP 0x26
#define GYRO_I2C_ID_OUT_X_L 0x28
#define GYRO_I2C_ID_OUT_Y_L 0x2A
#define GYRO_I2C_ID_OUT_Z_L 0x2C
#define GYRO_I2C_ID_CTRL_REG1 0x20
#define GYRO_I2C_ID_INT1_CFG 0x30
#define GYRO_I2C_ID_FIFO_CTRL_REG 0x2E
#define GYRO_I2C_ID_STATUS_REG 0x27
#define GYRO_I2C_ONALLAXIS 0xCF
#define GYRO_I2C_TIMEOUT 10
#define GYRO_I2C_STATUS_REG 0x27

#define ACSEL_I2C_ADRESS 0x32
#define ACSEL_I2C_ID_OUT_X_L 0x28
#define ACSEL_I2C_ID_Who_Am_I 0x0F
#define ACSEL_I2C_ID_CTRL_REG1 0x20
#define ACSEL_I2C_CTRL_REG1_SETTING 0x77
#define ACSEL_I2C_ID_CTRL_REG2 0x21
#define ACSEL_I2C_CTRL_REG2_SETTING 0xB0
#define ACSEL_I2C_ID_CTRL_REG3 0x22
#define ACSEL_I2C_CTRL_REG3_SETTING 0x00
#define ACSEL_I2C_ID_CTRL_REG4 0x23
#define ACSEL_I2C_CTRL_REG4_SETTING 0x28
#define ACSEL_I2C_ID_CTRL_REG5 0x24
#define ACSEL_I2C_CTRL_REG5_SETTING 0x00
#define ACSEL_I2C_ID_FIFO_CTRL_REG 0x2E
#define ACSEL_I2C_ONALLAXIS 0xFF
#define ACSEL_I2C_TIMEOUT 10

#define BOOT_TIME 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
xSemaphoreHandle ButtonSem;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim6;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for I2CScan */
osThreadId_t I2CScanHandle;
const osThreadAttr_t I2CScan_attributes = {
  .name = "I2CScan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ChannelControlT */
osThreadId_t ChannelControlTHandle;
const osThreadAttr_t ChannelControlT_attributes = {
  .name = "ChannelControlT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for IndicationTask */
osThreadId_t IndicationTaskHandle;
const osThreadAttr_t IndicationTask_attributes = {
  .name = "IndicationTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ButtonControlTa */
osThreadId_t ButtonControlTaHandle;
const osThreadAttr_t ButtonControlTa_attributes = {
  .name = "ButtonControlTa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CanControlTask */
osThreadId_t CanControlTaskHandle;
const osThreadAttr_t CanControlTask_attributes = {
  .name = "CanControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ControlModeDevT */
osThreadId_t ControlModeDevTHandle;
const osThreadAttr_t ControlModeDevT_attributes = {
  .name = "ControlModeDevT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for InclineControlT */
osThreadId_t InclineControlTHandle;
const osThreadAttr_t InclineControlT_attributes = {
  .name = "InclineControlT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TimeTask */
osThreadId_t TimeTaskHandle;
const osThreadAttr_t TimeTask_attributes = {
  .name = "TimeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GyroQueue */
osMessageQueueId_t GyroQueueHandle;
const osMessageQueueAttr_t GyroQueue_attributes = {
  .name = "GyroQueue"
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
osStaticSemaphoreDef_t myBinarySem01ControlBlock;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01",
  .cb_mem = &myBinarySem01ControlBlock,
  .cb_size = sizeof(myBinarySem01ControlBlock),
};
/* Definitions for myBinarySem02 */
osSemaphoreId_t myBinarySem02Handle;
const osSemaphoreAttr_t myBinarySem02_attributes = {
  .name = "myBinarySem02"
};
/* Definitions for GyroBinSem */
osSemaphoreId_t GyroBinSemHandle;
const osSemaphoreAttr_t GyroBinSem_attributes = {
  .name = "GyroBinSem"
};
/* Definitions for myCountingSem01 */
osSemaphoreId_t myCountingSem01Handle;
osStaticSemaphoreDef_t myCountingSem01ControlBlock;
const osSemaphoreAttr_t myCountingSem01_attributes = {
  .name = "myCountingSem01",
  .cb_mem = &myCountingSem01ControlBlock,
  .cb_size = sizeof(myCountingSem01ControlBlock),
};
/* Definitions for GyroCountingSem */
osSemaphoreId_t GyroCountingSemHandle;
const osSemaphoreAttr_t GyroCountingSem_attributes = {
  .name = "GyroCountingSem"
};
/* USER CODE BEGIN PV */





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_CAN1_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void *argument);
void I2CScanTask(void *argument);
void ChannelControl(void *argument);
void Indication(void *argument);
void ButtonControl(void *argument);
void CanControl(void *argument);
void ControlModeDev(void *argument);
void InclineControl(void *argument);
void TimeControl(void *argument);

/* USER CODE BEGIN PFP */
Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};
static int32_t i3g4250dtr_platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len);
static int32_t i3g4250dtr_platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);

static int32_t ii2s_dh_platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len);
//static int32_t ii2s_dh_platform_read(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len);

void AllMemEraze(void);


uint8_t tx_buffer[1000];

uint8_t regAddress = GYRO_I2C_ID_OUT_X_L;
uint8_t regData = 0;
uint8_t TxData;
uint8_t RxData;

uint8_t BuffFRAM[9];

uint8_t CanDataTxBuff[8];
uint8_t RxDataBuffer [10];
uint8_t TxDataBuffer [10];
uint8_t AcselRxDataBuffer [10];
uint8_t GyroRxDataBuffer [10];
GPIO_PinState EPV1,EPV2,EPV3,EPV4;


uint16_t GyroOutX, GyroOutY, GyroOutZ, GyroStatusReg,GyroOutTemp;
float fGyroOutX,fGyroOutY,fGyroOutZ;
uint16_t AcselOutX, AcselOutY, AcselOutZ,AcselStatusReg,AcselOutTemp;

static int16_t i3g4250Buff[3];
static int16_t iis2dsBuff[3];

uint16_t data_raw_angular_rate[3];

/*
static int8_t i3g4250Buff[6];
static int8_t iis2dsBuff[6];
*/
uint32_t TimerKalman;
uint8_t WhoAmI;
/*CAN Global frame data*/
uint32_t RxIDCurrentFrame;
uint16_t RxIDShort;
uint8_t RxDataCurrentFrameBuff [8];

struct ExtData RxDataFromBis;
struct ExtData EmptyStr;
//struct ExtData InsideData;
struct DevCondition DeviceData;
struct DataI2CPer Gyro;

	struct DataI2CPer Acsel;
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
	__HAL_RCC_I2C3_CLK_ENABLE();
	
	__HAL_RCC_I2C3_FORCE_RESET();

	__HAL_RCC_I2C3_RELEASE_RESET();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	RxDataFromBis = EmptyStr;
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_CAN1_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	
	//AllMemEraze();
	
	DownloadDataFromMem(&DeviceData);//подгружаем значени€ из пам€тиж
	
	ButtonSem = xSemaphoreCreateCounting(10,0);
	
	Gyro.Adress = GYRO_I2C_ADRESS;
	//Gyro.SETUP_CNTRL_REG = GYRO_I2C_ONALLAXIS; 
	
	Acsel.Adress = ACSEL_I2C_ADRESS;
	//Acsel.SETUP_CNTRL_REG = ACSEL_I2C_ONALLAXIS;
	//memset(*RxDataFromBis,)

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_ERROR|CAN_IT_BUSOFF|CAN_IT_LAST_ERROR_CODE);
	
	HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
		
	if(HAL_GPIO_ReadPin(Dir_pin_GPIO_Port,Dir_pin_Pin)==GPIO_PIN_SET)
	{
		/*если сторона установки левый борт*/
		DeviceData.Bort = Left;//левый борт
	}
	else
		DeviceData.Bort = Right; //правй борт
		//HAL_DMA_Start_IT();
		
		//HAL_DMA_Start();
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* creation of myBinarySem02 */
  myBinarySem02Handle = osSemaphoreNew(1, 1, &myBinarySem02_attributes);

  /* creation of GyroBinSem */
  GyroBinSemHandle = osSemaphoreNew(1, 1, &GyroBinSem_attributes);

  /* creation of myCountingSem01 */
  myCountingSem01Handle = osSemaphoreNew(10, 10, &myCountingSem01_attributes);

  /* creation of GyroCountingSem */
  GyroCountingSemHandle = osSemaphoreNew(2, 2, &GyroCountingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of GyroQueue */
  GyroQueueHandle = osMessageQueueNew (30, sizeof(uint16_t), &GyroQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of I2CScan */
  I2CScanHandle = osThreadNew(I2CScanTask, NULL, &I2CScan_attributes);

  /* creation of ChannelControlT */
  ChannelControlTHandle = osThreadNew(ChannelControl, NULL, &ChannelControlT_attributes);

  /* creation of IndicationTask */
  IndicationTaskHandle = osThreadNew(Indication, NULL, &IndicationTask_attributes);

  /* creation of ButtonControlTa */
  ButtonControlTaHandle = osThreadNew(ButtonControl, NULL, &ButtonControlTa_attributes);

  /* creation of CanControlTask */
  CanControlTaskHandle = osThreadNew(CanControl, NULL, &CanControlTask_attributes);

  /* creation of ControlModeDevT */
  ControlModeDevTHandle = osThreadNew(ControlModeDev, NULL, &ControlModeDevT_attributes);

  /* creation of InclineControlT */
  InclineControlTHandle = osThreadNew(InclineControl, NULL, &InclineControlT_attributes);

  /* creation of TimeTask */
  TimeTaskHandle = osThreadNew(TimeControl, NULL, &TimeTask_attributes);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	
	sFilterConfig.FilterBank=0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh=(uint16_t)(CAN_FILTER_ID_0>>13);
	sFilterConfig.FilterIdLow=(uint16_t)(CAN_FILTER_ID_0<<3)|0x04;
	sFilterConfig.FilterMaskIdHigh=(uint16_t)(CAN_FILTER_MASK_0>>13);
	sFilterConfig.FilterMaskIdLow=(uint16_t)(CAN_FILTER_MASK_0<<3)|0x04;
	sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
	sFilterConfig.FilterActivation=ENABLE;
	sFilterConfig.SlaveStartFilterBank=15;
	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
	{
		Error_Handler();
	}
	
	sFilterConfig.FilterBank=1;
	sFilterConfig.FilterIdHigh=(uint16_t)(CAN_FILTER_ID_1>>13);
	sFilterConfig.FilterIdLow=(uint16_t)(CAN_FILTER_ID_1<<3)|0x04;
	sFilterConfig.FilterMaskIdHigh=(uint16_t)(CAN_FILTER_MASK_1>>13);
	sFilterConfig.FilterMaskIdLow=(uint16_t)(CAN_FILTER_MASK_1<<3)|0x04;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
	{
		Error_Handler();
	}
		sFilterConfig.FilterBank=2;
	sFilterConfig.FilterIdHigh=(uint16_t)(CAN_FILTER_ID_2<<5);
	sFilterConfig.FilterIdLow=0x0000;
	sFilterConfig.FilterMaskIdHigh=(uint16_t)(CAN_FILTER_MASK_2<<5);
	sFilterConfig.FilterMaskIdLow=0x0000;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
	{
		Error_Handler();
	}

  /* USER CODE END CAN1_Init 2 */

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

		HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
	
	/* I2C2_TX Init */
  
  /* USER CODE END I2C2_Init 2 */

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
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ch1_1_Pin|ch1_2_Pin|ch2_1_Pin|ch2_2_Pin
                          |ch3_1_Pin|ch3_2_Pin|ch4_1_Pin|ch4_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, KeyOn1_Pin|KeyOn2_Pin|KeyOn3_Pin|KeyOn4_Pin
                          |on1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(on2_GPIO_Port, on2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, f2_Pin|f1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SigKz1_Pin SigKz2_Pin SigKz3_Pin SigKz4_Pin
                           Button_Pin DB_Pin ZS_Pin */
  GPIO_InitStruct.Pin = SigKz1_Pin|SigKz2_Pin|SigKz3_Pin|SigKz4_Pin
                          |Button_Pin|DB_Pin|ZS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ch1_1_Pin ch1_2_Pin ch2_1_Pin ch2_2_Pin
                           ch3_1_Pin ch3_2_Pin ch4_1_Pin ch4_2_Pin */
  GPIO_InitStruct.Pin = ch1_1_Pin|ch1_2_Pin|ch2_1_Pin|ch2_2_Pin
                          |ch3_1_Pin|ch3_2_Pin|ch4_1_Pin|ch4_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KeyOn1_Pin KeyOn2_Pin KeyOn3_Pin KeyOn4_Pin
                           on1_Pin */
  GPIO_InitStruct.Pin = KeyOn1_Pin|KeyOn2_Pin|KeyOn3_Pin|KeyOn4_Pin
                          |on1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : G_DR_Pin G_INT_Pin */
  GPIO_InitStruct.Pin = G_DR_Pin|G_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EPV_ON1_Pin EPV_ON2_Pin EPV_ON3_Pin EPV_ON4_Pin */
  GPIO_InitStruct.Pin = EPV_ON1_Pin|EPV_ON2_Pin|EPV_ON3_Pin|EPV_ON4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A_INT1_Pin A_INT2_Pin */
  GPIO_InitStruct.Pin = A_INT1_Pin|A_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : on2_Pin */
  GPIO_InitStruct.Pin = on2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(on2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : f2_Pin f1_Pin */
  GPIO_InitStruct.Pin = f2_Pin|f1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Dir_pin_Pin */
  GPIO_InitStruct.Pin = Dir_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Dir_pin_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void AllMemEraze(void)
{
		uint16_t NomerBloka;//—ерийный номер блока
	uint16_t YearOfManfacture;//год выпуска
	uint8_t MounthOfManafacture;//мес€ц выпуска
	uint8_t DayOfManafacture;
	DeviceData.NomerBloka = 0x0C;
	DeviceData.MounthOfManafacture = 7;
	DeviceData.DayOfManafacture = 19;
	DeviceData.YearOfManafacture = 2022;
	DeviceData.EPV1Count = 0x01;
	DeviceData.EPV2Count = 0xFF;
	DeviceData.EPV3Count = 0xFFF;
	DeviceData.EPV4Count = 0xFFFFF;
	DeviceData.DistanceTraveled = 0;
	DeviceData.TimeMinuts = 0;
	DeviceData.TimeOfWork = 0;
	WrireToMemUint32(DeviceData.EPV1Count,FRAM_MEM_ID_EPV1);
	WrireToMemUint32(DeviceData.EPV2Count,FRAM_MEM_ID_EPV2);
	WrireToMemUint32(DeviceData.EPV3Count,FRAM_MEM_ID_EPV3);
	WrireToMemUint32(DeviceData.EPV4Count,FRAM_MEM_ID_EPV4);
	WrireToMemUint32(DeviceData.TimeMinuts,FRAM_MEM_ID_TIME_MIN);

	WrireToMemUint32(DeviceData.TimeOfWork,FRAM_MEM_ID_TIME);
	
		WrireToMemUint32(DeviceData.DistanceTraveled,FRAM_MEM_ID_DISTANCE);
		
		WrireToMemUint16(DeviceData.NomerBloka,FRAM_MEM_ID_SER_NUM);
		WrireToMemUint8(DeviceData.MounthOfManafacture,FRAM_MEM_ID_DATA_MOUNTH);
		WrireToMemUint8(DeviceData.DayOfManafacture,FRAM_MEM_ID_DATA_DAY);
		WrireToMemUint16(DeviceData.YearOfManafacture,FRAM_MEM_ID_DATA_YEAR);
}

void dma_m2m_callback(DMA_HandleTypeDef *hdma_memtomem_dma2_stream0)
{
	
}


void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef *hi2c)
{
	CHANNEL_1_LED_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxDataCurrentFrameBuff)==HAL_OK)
	{
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_8);
		//if(RxHeader.ExtId!=0)
		RxIDCurrentFrame = RxHeader.ExtId;
		RxHeader.ExtId = 0;
		//if(RxHeader.StdId!=0)
		RxIDShort = RxHeader.StdId;
		RxHeader.StdId = 0;
	}
}
/*---------------------------------------------------*/
//«авершена передача всех данных
void HAL_I2C_TxCpltCallback(I2C_HandleTypeDef *i2c)
{
	if(i2c == &hi2c2)
	{
		HAL_GPIO_TogglePin(ch1_1_GPIO_Port,ch1_1_Pin);
		HAL_GPIO_TogglePin(ch1_2_GPIO_Port,ch1_2_Pin);
	}
}
/*---------------------------------------------------*/
//«авершена передача половина данных
void HAL_I2C_TxHalfCallback(I2C_HandleTypeDef *i2c)
{
	if(i2c == &hi2c2)
	{
		HAL_GPIO_TogglePin(ch2_1_GPIO_Port,ch1_1_Pin);
		HAL_GPIO_TogglePin(ch2_2_GPIO_Port,ch1_2_Pin);
	}
}


static int32_t i3g4250dtr_platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len)
{
	Reg |= 0x80;
  //HAL_I2C_Mem_Write_DMA(handle, I3G4250D_I2C_ADD_L, Reg,I2C_MEMADD_SIZE_8BIT, (uint8_t*) Bufp, len);
	HAL_I2C_Mem_Write(handle, I3G4250D_I2C_ADD_L, Reg,I2C_MEMADD_SIZE_8BIT, (uint8_t*) Bufp, len,1);
}
static int32_t i3g4250dtr_platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
	 Reg |= 0x80;
 // HAL_I2C_Mem_Read_DMA(handle, I3G4250D_I2C_ADD_L, Reg,I2C_MEMADD_SIZE_8BIT, Bufp, len);
	HAL_I2C_Mem_Read(handle, I3G4250D_I2C_ADD_L, Reg,I2C_MEMADD_SIZE_8BIT, Bufp, len,1);
	
}
static int32_t ii2s_dh_platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len)
{
	Reg |= 0x80;
  //HAL_I2C_Mem_Write_DMA(handle, IIS2DH_I2C_ADD_L, Reg,I2C_MEMADD_SIZE_8BIT, (uint8_t*) Bufp, len);
	HAL_I2C_Mem_Write(handle, IIS2DH_I2C_ADD_L, Reg,I2C_MEMADD_SIZE_8BIT, (uint8_t*) Bufp, len,1);
}
static int32_t ii2s_dh_platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
  Reg |= 0x80;
  //HAL_I2C_Mem_Read_DMA(handle, IIS2DH_I2C_ADD_L, Reg,I2C_MEMADD_SIZE_8BIT, Bufp, len);
	HAL_I2C_Mem_Read(handle, IIS2DH_I2C_ADD_L, Reg,I2C_MEMADD_SIZE_8BIT, Bufp, len,1);
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
	//DeviceData.DistanceTraveled = 0;
	//WrireToMemUint32(DeviceData.DistanceTraveled,FRAM_MEM_ID_DISTANCE);
	 DeviceData.DistanceTraveled = ReadFromMemUint32(FRAM_MEM_ID_DISTANCE);

  /* Infinite loop */
  for(;;)
  {
		if(DeviceData.Mode != ZapretSmazki)		
			DeviceData.ActualDistance += DistanseCount(DeviceData.Speed,1);
		else
			DeviceData.ActualDistance = 0;
		/*-----------------------------------------------------------------*/
		DeviceData.FullDistace += DistanseCount(DeviceData.Speed,1);
			if(DeviceData.FullDistace >= 1000)
			{
				DeviceData.FullDistace = 0x00;
				DeviceData.DistanceTraveled ++;//приращиваем пройденный путь в км
				WrireToMemUint32(DeviceData.DistanceTraveled,FRAM_MEM_ID_DISTANCE);
				
			}
			osDelay(100);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_I2CScanTask */
/* USER CODE END Header_I2CScanTask */
void I2CScanTask(void *argument)
{
  /* USER CODE BEGIN I2CScanTask */
	
	osDelay(10);
	for(;;)
	{
		
		 
		osDelay(100);
	}
  /* USER CODE END I2CScanTask */
}


/* USER CODE BEGIN Header_ChannelControl */
/**
* @brief Function implementing the ChannelControlT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChannelControl */
void ChannelControl(void *argument)
{
  /* USER CODE BEGIN ChannelControl */
	DeviceData.Mode = ZapretSmazki;
	DeviceData.EPV1Count = ReadFromMemUint32(FRAM_MEM_ID_EPV1);
	DeviceData.EPV2Count = ReadFromMemUint32(FRAM_MEM_ID_EPV2);
	DeviceData.EPV3Count = ReadFromMemUint32(FRAM_MEM_ID_EPV3);
	DeviceData.EPV4Count = ReadFromMemUint32(FRAM_MEM_ID_EPV4);
	DeviceData.TimeBetweenInjection = 1;
		
  /* Infinite loop */
  for(;;)
  {
		ChannelControlSwitch(&DeviceData);

		//osDelay(DeviceData.TimeBetweenInjection * 100);
		osDelay(1);
  }
  /* USER CODE END ChannelControl */
}

/* USER CODE BEGIN Header_Indication */
/**
* @brief Function implementing the IndicationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Indication */
void Indication(void *argument)
{
  /* USER CODE BEGIN Indication */
	STATUS_LED_GREEN;
		//uint8_t Buff[9];
  /* Infinite loop */
  for(;;)
  {
		
		if(DeviceData.KZ_ERROR)
		{
				STATUS_LED_RED;	
		}
		else
		{
				if(DeviceData.OBR_ERROR|DeviceData.CAN_ERROR == 2)
					STATUS_LED_YELLOW;
				if((!DeviceData.KZ_ERROR)&(!DeviceData.OBR_ERROR)&(DeviceData.CAN_ERROR != 2))
				{
					STATUS_LED_GREEN;
				}
				
		}
		
			
		osDelay(100);
  }
  /* USER CODE END Indication */
}

/* USER CODE BEGIN Header_ButtonControl */
/**
* @brief Function implementing the ButtonControlTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ButtonControl */
void ButtonControl(void *argument)
{
  /* USER CODE BEGIN ButtonControl */

	//uint8_t ButtonCondition = 0;
  /* Infinite loop */
  for(;;)
  {
		if(DeviceData.Mode != SelfTest)
		if(HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin) == GPIO_PIN_RESET)
		{
			osDelay(100);
			if(HAL_GPIO_ReadPin(Button_GPIO_Port,Button_Pin) == GPIO_PIN_RESET)
			{
				DeviceData.Mode = SelfTest;
				osDelay(500);
			}
		}
		
		osDelay(10);
  }
  /* USER CODE END ButtonControl */
}

/* USER CODE BEGIN Header_CanControl */
/**
* @brief Function implementing the CanControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanControl */
void CanControl(void *argument)
{
  /* USER CODE BEGIN CanControl */
	/*----предварительна€ настройка структуры, обнул€ем результаты и подгружаем из пам€ти FRAM---------------*/
	RxDataFromBis = EmptyStr;
	RxIDCurrentFrame = 0;
	uint8_t OldError = 0;
	uint8_t oldValZs1 = RxDataFromBis.zs1;
	uint8_t oldValBd = RxDataFromBis.Bak;
	//DeviceData.KZ_ERROR = 0X04;
	//DeviceData.OBR_ERROR = 0x05;
	//RxDataFromBis = GetDataFromMem(RxDataFromBis);
  /* Infinite loop */
  for(;;)
  {
			if(RxIDShort)
			{
					if(RxDataCurrentFrameBuff[0]==0x01)
						DeviceData.ActivExt = EnableSw;
					else 
						DeviceData.ActivExt = DisableSw;
						
					RxIDShort=0;
			}		
		if(RxIDCurrentFrame)
		{
				IntreputDataFromBis(RxIDCurrentFrame,RxDataCurrentFrameBuff,&RxDataFromBis, &DeviceData);
				if(RxIDCurrentFrame!=0)
					DeviceData.CAN_ERROR = 0;
				RxIDCurrentFrame = 0;


				if(RxDataFromBis.T2)//“ормоз
				{
					//RxDataFromBis.T2 = 0;
					//DeviceData.Mode = ZapretSmazki;
				}
				if(RxDataFromBis.T3)//ѕесок
				{
					//RxDataFromBis.T3 = 0;
					//DeviceData.Mode = ZapretSmazki;
				}
				if(RxDataFromBis.geroinit == 1)//ѕровести инициализацию гироскопа
				{
					
				}
				if(RxDataFromBis.contr == 1)
				{
					DeviceData.Mode = SelfTest;
					RxDataFromBis.contr = 0;
				}
				if(RxDataFromBis.error_log == 1)//ѕередать лог ошибок модул€
				{
					CanSendExtMessage(CAN_PROT_IDExt_FORM_MUGS_03_ERRORS,CanDataTxBuff);
					RxDataFromBis.error_log = 0;
				}
				
			}
				if(RxDataFromBis.info == 1)//передать информацию о блоке
				{
					
					DeviceData.DayOfManafacture = ReadFromMemUint8(FRAM_MEM_ID_DATA_DAY);
					DeviceData.MounthOfManafacture = ReadFromMemUint8(FRAM_MEM_ID_DATA_MOUNTH);
					DeviceData.YearOfManafacture = ReadFromMemUint16(FRAM_MEM_ID_DATA_YEAR);
					DeviceData.NomerBloka = ReadFromMemUint16(FRAM_MEM_ID_SER_NUM);
					MessageFormateToBis(CAN_PROT_IDExt_FORM_MUGS_03_INFO,CanDataTxBuff,&RxDataFromBis,&DeviceData);
					CanSendExtMessage(CAN_PROT_IDExt_FORM_MUGS_03_INFO,CanDataTxBuff);
					MessageFormateToBis(CAN_PROT_IDExt_FORM_MUGS_03_ERRORS,CanDataTxBuff,&RxDataFromBis,&DeviceData);
					CanSendExtMessage(CAN_PROT_IDExt_FORM_MUGS_03_ERRORS,CanDataTxBuff);
					RxDataFromBis.info = 0;
					osDelay(10);
					//	MessageFormateToBis(CAN_PROT_IDExt_FORM_MUGS_03_INFO,CanDataTxBuff,RxDataFromBis);
						
				}
				
				if(RxDataFromBis.cnt == ReciveCommand)//передать состо€ние о счетчиках
				{
					
					MessageFormateToBis(CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV1_2,CanDataTxBuff,&RxDataFromBis ,&DeviceData);
					CanSendExtMessage(CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV1_2,CanDataTxBuff);
					MessageFormateToBis(CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV3_4,CanDataTxBuff,&RxDataFromBis ,&DeviceData);
					CanSendExtMessage(CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV3_4,CanDataTxBuff);
					MessageFormateToBis(CAN_PROT_IDExt_FORM_MUGS_03_INFO_TIME,CanDataTxBuff,&RxDataFromBis ,&DeviceData);
					CanSendExtMessage(CAN_PROT_IDExt_FORM_MUGS_03_INFO_TIME,CanDataTxBuff);
					RxDataFromBis.cnt = NoCommand;
				}
		//отправка сообщений сразу после короткого замыкани€
		if(DeviceData.KZ_ERROR!= OldError)
		{
			MessageFormateToBis(CAN_PROT_IDExt_FORM_MUGS_03_ERRORS,CanDataTxBuff,&RxDataFromBis,&DeviceData);
			CanSendExtMessage(CAN_PROT_IDExt_FORM_MUGS_03_ERRORS,CanDataTxBuff);
			OldError = DeviceData.KZ_ERROR;
		}
		//отправка сообщений сразу по приходу внешнего сигнала - запрета смазки и бака
		if((RxDataFromBis.zs1 != oldValZs1)|(RxDataFromBis.Bak != oldValBd))
		{
			oldValZs1 = RxDataFromBis.zs1;
			oldValBd = RxDataFromBis.Bak;
			MessageFormateToBis(CAN_PROT_IDExt_FORM_MUGS_03_ERRORS,TxDataBuffer,&RxDataFromBis,&DeviceData);
			CanSendExtMessage(CAN_PROT_IDExt_FORM_MUGS_03_ERRORS,TxDataBuffer);
		}
		osDelay(1);
  }
  /* USER CODE END CanControl */
}

/* USER CODE BEGIN Header_ControlModeDev */
/**
* @brief Function implementing the ControlModeDevT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlModeDev */
void ControlModeDev(void *argument)
{
  /* USER CODE BEGIN ControlModeDev */
	DeviceData.Mode = ZapretSmazki;
	DeviceData.Area = Linage;
	DeviceData.DistanceL1 = 100;
	DeviceData.DistanceL2 = 300;
	DeviceData.DistanceL3 = 200;
	DeviceData.Speed = 0;
	DeviceData.SpeedV1 = 5;
	DeviceData.SpeedV2 = 55;
	DeviceData.Time_Injection = 4;
	
  /* Infinite loop */
  for(;;)
  {
		if(DeviceData.Mode!=SelfTest)
		{
			if(HAL_GPIO_ReadPin(ZS_GPIO_Port,ZS_Pin)==GPIO_PIN_SET)
			{
			
				RxDataFromBis.zs1 = ReciveCommand;
				DeviceData.Mode = ZapretSmazki;
			}
			else
			{
				RxDataFromBis.zs1 = NoCommand;
				
				if(DeviceData.Direction && (DeviceData.Area == Linage) && 
				(DeviceData.Speed > DeviceData.SpeedV1)&&
				(DeviceData.Speed < DeviceData.SpeedV2) &&
				(DeviceData.ActivExt == EnableSw)&&
				(RxDataFromBis.T2 ==0)&&
				(RxDataFromBis.T3 ==0))
				{
					DeviceData.Mode = ActivSmazkaL1;
				}	
				if(DeviceData.Direction && (DeviceData.Area == Linage) && 
					(DeviceData.Speed > DeviceData.SpeedV2) &&
				 (DeviceData.ActivExt == EnableSw)&&
					(RxDataFromBis.T2 ==0)&&
					(RxDataFromBis.T3 ==0))
				{
					DeviceData.Mode = ActivSmazkaL2;
				}
				if(DeviceData.Direction && (DeviceData.Area == NonLinage) &&
					(DeviceData.Speed > DeviceData.SpeedV1)&&
				(DeviceData.ActivExt == EnableSw) &&
				(RxDataFromBis.T2 ==0)&&
				(RxDataFromBis.T3 ==0))
				{
					DeviceData.Mode = ActivSmazkaL3;
				}
				if((DeviceData.Direction)&&(DeviceData.Speed < DeviceData.SpeedV1)&&
				(DeviceData.ActivExt == EnableSw))
				{
					DeviceData.Mode = ZapretSmazki;
				}
				if(DeviceData.Direction != 1)
				{
					DeviceData.Mode = ZapretSmazki;
				}
				if(DeviceData.ActivExt == DisableSw)
				{
					DeviceData.Mode = ZapretSmazki;
				}
				if((RxDataFromBis.T2)||(RxDataFromBis.T3))
				{
					DeviceData.Mode = ZapretSmazki;
				}
			}		
		}
		else if(DeviceData.Mode == SelfTest)
		{
			
		}
		if(HAL_GPIO_ReadPin(DB_GPIO_Port,DB_Pin)==GPIO_PIN_SET)
		{
			RxDataFromBis.Bak = ReciveCommand;
		}
		else
			RxDataFromBis.Bak = NoCommand;
	    osDelay(200);
  }
  /* USER CODE END ControlModeDev */
}

/* USER CODE BEGIN Header_InclineControl */
/**
* @brief Function implementing the InclineControlT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InclineControl */
void InclineControl(void *argument)
{
  /* USER CODE BEGIN InclineControl */
	/*-----------------------------------------*/
	/*-переделать начальный уровень в начале самотестировани€?-*/
	
	//DownloadDataFromMem(&DeviceData);
DeviceData.CAN_ERROR = 0;
	/*-----------------------------------------*/
  /* Infinite loop */
  for(;;)
  {
		if(DeviceData.CAN_ERROR == 0)
					DeviceData.CAN_ERROR = 1;
				if(DeviceData.CAN_ERROR == 1)
				{
					osDelay(10000);
					if(DeviceData.CAN_ERROR != 0)
						DeviceData.CAN_ERROR = 2;
				}
    osDelay(100);
  }
  /* USER CODE END InclineControl */
}

/* USER CODE BEGIN Header_TimeControl */
/**
* @brief Function implementing the TimeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TimeControl */
void TimeControl(void *argument)
{
  /* USER CODE BEGIN TimeControl */
	/*DeviceData.TimeOfWork = 0;
	WrireToMemUint32(DeviceData.TimeOfWork,FRAM_MEM_ID_TIME);
	DeviceData.TimeMinuts = 0;
	WrireToMemUint32(DeviceData.TimeMinuts,FRAM_MEM_ID_TIME_MIN);
	*/
	DeviceData.TimeOfWork = ReadFromMemUint32(FRAM_MEM_ID_TIME);
	DeviceData.TimeMinuts = ReadFromMemUint32(FRAM_MEM_ID_TIME_MIN);
	//WrireToMemUint32(DeviceData.TimeMinuts,FRAM_MEM_ID_TIME_MIN);
  /* Infinite loop */
  for(;;)
  {
    osDelay(60000);
		
		DeviceData.TimeMinuts ++;
		WrireToMemUint32(DeviceData.TimeMinuts,FRAM_MEM_ID_TIME_MIN);
		if(DeviceData.TimeMinuts>=60)
		{
			DeviceData.TimeMinuts = 0x00;
			DeviceData.TimeOfWork++;
			WrireToMemUint32(DeviceData.TimeOfWork,FRAM_MEM_ID_TIME);
		}
  }
  /* USER CODE END TimeControl */
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

