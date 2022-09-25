#ifndef NOZZEL_CONTROL_H
#define NOZZEL_CONTROL_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "I2C_Dev.h"

#define CHANNEL_1 0x01
#define CHANNEL_2 0x02
#define CHANNEL_3 0x04
#define CHANNEL_4 0x08

#define CHANNEL_1_LED_OK HAL_GPIO_WritePin(ch1_1_GPIO_Port, ch1_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(ch1_2_GPIO_Port, ch1_2_Pin, GPIO_PIN_SET);
#define CHANNEL_1_LED_OFF HAL_GPIO_WritePin(ch1_1_GPIO_Port, ch1_1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(ch1_2_GPIO_Port, ch1_2_Pin, GPIO_PIN_RESET);
#define CHANNEL_2_LED_OK HAL_GPIO_WritePin(ch2_1_GPIO_Port, ch2_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(ch2_2_GPIO_Port, ch2_2_Pin, GPIO_PIN_SET);
#define CHANNEL_2_LED_OFF HAL_GPIO_WritePin(ch2_1_GPIO_Port, ch2_1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(ch2_2_GPIO_Port, ch2_2_Pin, GPIO_PIN_RESET);
#define CHANNEL_3_LED_OK HAL_GPIO_WritePin(ch3_1_GPIO_Port, ch3_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(ch3_2_GPIO_Port, ch3_2_Pin, GPIO_PIN_SET);
#define CHANNEL_3_LED_OFF HAL_GPIO_WritePin(ch3_1_GPIO_Port, ch3_1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(ch3_2_GPIO_Port, ch3_2_Pin, GPIO_PIN_RESET);
#define CHANNEL_4_LED_OK HAL_GPIO_WritePin(ch4_1_GPIO_Port, ch4_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(ch4_2_GPIO_Port, ch4_2_Pin, GPIO_PIN_SET);
#define CHANNEL_4_LED_OFF HAL_GPIO_WritePin(ch4_1_GPIO_Port, ch4_1_Pin, GPIO_PIN_RESET);HAL_GPIO_WritePin(ch4_2_GPIO_Port, ch4_2_Pin, GPIO_PIN_RESET);

#define CHANNEL_1_POWER_ON HAL_GPIO_WritePin(KeyOn1_GPIO_Port,KeyOn1_Pin,GPIO_PIN_SET);
#define CHANNEL_1_POWER_OFF HAL_GPIO_WritePin(KeyOn1_GPIO_Port,KeyOn1_Pin,GPIO_PIN_RESET);
#define CHANNEL_2_POWER_ON HAL_GPIO_WritePin(KeyOn2_GPIO_Port,KeyOn2_Pin,GPIO_PIN_SET);
#define CHANNEL_2_POWER_OFF HAL_GPIO_WritePin(KeyOn2_GPIO_Port,KeyOn2_Pin,GPIO_PIN_RESET);
#define CHANNEL_3_POWER_ON HAL_GPIO_WritePin(KeyOn3_GPIO_Port,KeyOn3_Pin,GPIO_PIN_SET);
#define CHANNEL_3_POWER_OFF HAL_GPIO_WritePin(KeyOn3_GPIO_Port,KeyOn3_Pin,GPIO_PIN_RESET);
#define CHANNEL_4_POWER_ON HAL_GPIO_WritePin(KeyOn4_GPIO_Port,KeyOn4_Pin,GPIO_PIN_SET);
#define CHANNEL_4_POWER_OFF HAL_GPIO_WritePin(KeyOn4_GPIO_Port,KeyOn4_Pin,GPIO_PIN_RESET);

#define SELF_TEST_TIME_ON 2000 //мс
#define SELF_TEST_TIME_OFF 2000 //мс

typedef enum 
{
	ChannelOn = 0,
	ChannelShortCircuit,
	ChannelBreak,
}ChannelState;

typedef enum
{
	NoChannel = 0,
	ExitChannel1,
	ExitChannel2,
	ExitChannel3,
	ExitChannel4
}NumberOfChannel;


struct NozzelControlData
{
	ChannelState State;
	NumberOfChannel Channel;
};

struct DevCondition; 

void NozzelAllOn (struct DevCondition *DevData);
void NozzelAllOff (struct DevCondition *DevData);
void NozzelSingleOn (NumberOfChannel Ch);
void NozzelSingleOff (NumberOfChannel Ch);
void ChannelIndication (void);
void ChannelControlSwitch(struct DevCondition *DevData);
uint8_t OverCurrentSigCheck (void);
uint8_t OutSigCheck (void);
ChannelState GetChannelState (NumberOfChannel Ch);
float DistanseCount(uint16_t Speed,int Time);
/*-------------------*/

void NozzelContr(struct DevCondition *DevData,uint8_t *ChannelCondition);


#endif