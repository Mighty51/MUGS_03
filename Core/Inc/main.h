/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../conf.h"

#include "I2C_Dev.h"
#include "NozzelControl.h"
#include "semphr.h"
#include "CanProt.h"

#include "i3g4250d_reg.h"
#include "iis2dh_reg.h"
#include "ModeDes.h"
//#include "../kalman/kalman.h"
#include "../FM24VN10/FM24VN10.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SigKz1_Pin GPIO_PIN_0
#define SigKz1_GPIO_Port GPIOC
#define SigKz2_Pin GPIO_PIN_1
#define SigKz2_GPIO_Port GPIOC
#define SigKz3_Pin GPIO_PIN_2
#define SigKz3_GPIO_Port GPIOC
#define SigKz4_Pin GPIO_PIN_3
#define SigKz4_GPIO_Port GPIOC
#define ch1_1_Pin GPIO_PIN_0
#define ch1_1_GPIO_Port GPIOA
#define ch1_2_Pin GPIO_PIN_1
#define ch1_2_GPIO_Port GPIOA
#define ch2_1_Pin GPIO_PIN_2
#define ch2_1_GPIO_Port GPIOA
#define ch2_2_Pin GPIO_PIN_3
#define ch2_2_GPIO_Port GPIOA
#define ch3_1_Pin GPIO_PIN_4
#define ch3_1_GPIO_Port GPIOA
#define ch3_2_Pin GPIO_PIN_5
#define ch3_2_GPIO_Port GPIOA
#define ch4_1_Pin GPIO_PIN_6
#define ch4_1_GPIO_Port GPIOA
#define ch4_2_Pin GPIO_PIN_7
#define ch4_2_GPIO_Port GPIOA
#define KeyOn1_Pin GPIO_PIN_4
#define KeyOn1_GPIO_Port GPIOC
#define KeyOn2_Pin GPIO_PIN_5
#define KeyOn2_GPIO_Port GPIOC
#define G_DR_Pin GPIO_PIN_0
#define G_DR_GPIO_Port GPIOB
#define G_DR_EXTI_IRQn EXTI0_IRQn
#define G_INT_Pin GPIO_PIN_1
#define G_INT_GPIO_Port GPIOB
#define G_INT_EXTI_IRQn EXTI1_IRQn
#define EPV_ON1_Pin GPIO_PIN_12
#define EPV_ON1_GPIO_Port GPIOB
#define EPV_ON2_Pin GPIO_PIN_13
#define EPV_ON2_GPIO_Port GPIOB
#define EPV_ON3_Pin GPIO_PIN_14
#define EPV_ON3_GPIO_Port GPIOB
#define EPV_ON4_Pin GPIO_PIN_15
#define EPV_ON4_GPIO_Port GPIOB
#define KeyOn3_Pin GPIO_PIN_6
#define KeyOn3_GPIO_Port GPIOC
#define KeyOn4_Pin GPIO_PIN_7
#define KeyOn4_GPIO_Port GPIOC
#define Button_Pin GPIO_PIN_8
#define Button_GPIO_Port GPIOC
#define A_INT1_Pin GPIO_PIN_9
#define A_INT1_GPIO_Port GPIOA
#define A_INT1_EXTI_IRQn EXTI9_5_IRQn
#define A_INT2_Pin GPIO_PIN_10
#define A_INT2_GPIO_Port GPIOA
#define A_INT2_EXTI_IRQn EXTI15_10_IRQn
#define DB_Pin GPIO_PIN_10
#define DB_GPIO_Port GPIOC
#define ZS_Pin GPIO_PIN_11
#define ZS_GPIO_Port GPIOC
#define on1_Pin GPIO_PIN_12
#define on1_GPIO_Port GPIOC
#define on2_Pin GPIO_PIN_2
#define on2_GPIO_Port GPIOD
#define f2_Pin GPIO_PIN_5
#define f2_GPIO_Port GPIOB
#define f1_Pin GPIO_PIN_8
#define f1_GPIO_Port GPIOB
#define Dir_pin_Pin GPIO_PIN_9
#define Dir_pin_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SETUP_SPEED_V1 0x0032 //����������� �������� �������� ������� ��� ��������� ��������
#define SETUP_SPEED_V2 0x0230 //������������ �������� �������� ������� ��� ��������� �������� � ������ ������
#define TIME_SPEED_COUNT 10//������������� ���������� ����������� ����������, �
#define TIME_INJECTION 40 //����� ������ � 10x ����������

#define DISTANCE_L1 30 //��������� ������� L1,�
#define DISTANCE_L2 120 //��������� ������� L2,�
#define DISTANCE_L3 40 //��������� ������� L3,�

/*---------GYRO&ACSEL----------------------*/
#define GYRO_ACTIV_LAVEL 10
#define ACSEL_ACTIV_LAVEL 10
#define RAD_TO_DEG 57.295779513082320876798154814105

typedef enum
{
	Linage = 0,
	NonLinage
}TrafficArea;
typedef enum
{
	Left = 0,
	Right,
}SetupSide;

typedef enum
{
	SelfTest = 0,
	ActivSmazkaL1,
	ActivSmazkaL2,
	ActivSmazkaL3,
	ZapretSmazki,
	ErrorCAN,
	ErrorOut,
}CurrentDeviceMode;

typedef enum
{
	DisableSw = 0,
	EnableSw
}Switch;

struct DevCondition
{
	Switch ActivExt;//��������� ���������� �� CAN ������� ������������ ID frame
	uint8_t Direction;//0-����� // 1 - ������
	CurrentDeviceMode Mode;
	TrafficArea Area;
	float Acceleration;
	float Speed;
	
	float ActualDistance;//���������� ���� � ���������� ������ �� ������
	
	uint32_t FullDistace;//������ ���������� ����
	
	float DistanceL1,DistanceL2,DistanceL3;
	float SpeedV1, SpeedV2;
	uint16_t Forsunki_Count;
	
	uint16_t NomerBloka;//�������� ����� �����
	uint16_t YearOfManfacture;//��� �������
	uint8_t MounthOfManafacture;//����� �������
	uint8_t DayOfManafacture;//���� �������
	
	uint32_t EPV1Count,EPV2Count,EPV3Count,EPV4Count;//�������� ��������� ��������
	uint32_t ProidenoeRasstoyaniye;//���������� ����������, ��
	uint32_t TimeOfWork;//��������� �� �����, �
	
	uint8_t Time_Injection;//����� �������
	float TimeBetweenInjection;//����������� ����� ����� ���������
	float PorogAccel;//��������� ���������
	uint8_t TimeBeforeKontrol;//����� �������� ����� ������� ������� ��������
	SetupSide Bort; // 0 - ����� ����, 1 - ������ ����
	uint8_t OBR_ERROR;//����� ������ 1 - ������ 0 - �� 
	// 1 - ��� 1 -��� ����� 2bit - 2ch...4bit-4ch
	uint8_t KZ_ERROR;//�������� ��������� ������ 1 - ������ 0 - �� 
	// 1 - ��� 1 -��� ����� 2bit - 2ch...4bit-4ch
	uint8_t En_Ch;//������������ ������ 
	//1-��� - 1-�� �����
	//2-��� - 2-�� �����
	//...
	//4-��� - 4-�� �����
	
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
