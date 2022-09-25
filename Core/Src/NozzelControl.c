#include "NozzelControl.h"

void ChannelControlSwitch(struct DevCondition *DevData)
{
	if(DevData->Mode!=ZapretSmazki)
	{
		switch(DevData->Mode){
			case ActivSmazkaL1:
				if((DevData->ActualDistance)>(DevData->DistanceL1))
				{
					if(HAL_GPIO_ReadPin(EPV_ON1_GPIO_Port,EPV_ON1_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR|=0x01;
					else
							DevData->OBR_ERROR&=0x0E;
					if(HAL_GPIO_ReadPin(EPV_ON2_GPIO_Port,EPV_ON2_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR |=0x02;
					else
							DevData->OBR_ERROR &=0x0D;
					if(HAL_GPIO_ReadPin(EPV_ON3_GPIO_Port,EPV_ON3_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR |=0x04;
					else
							DevData->OBR_ERROR &=0x0B;
					if(HAL_GPIO_ReadPin(EPV_ON4_GPIO_Port,EPV_ON4_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR |=0x08;
					else
							DevData->OBR_ERROR &=0x07;
					NozzelAllOn(DevData);
					osDelay(20);
					if(HAL_GPIO_ReadPin(SigKz1_GPIO_Port,SigKz1_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x01;
					if(HAL_GPIO_ReadPin(SigKz2_GPIO_Port,SigKz2_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x02;
					if(HAL_GPIO_ReadPin(SigKz3_GPIO_Port,SigKz3_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x04;
					if(HAL_GPIO_ReadPin(SigKz4_GPIO_Port,SigKz4_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x08;
					
					DevData->ActualDistance = 0;//обнуляем расстояние пройденное до впрыска
					osDelay((((uint8_t)DevData->Time_Injection)*100)-20);
					NozzelAllOff(DevData);
				}
				
				break;
			case ActivSmazkaL2:
				if(DevData->ActualDistance>=DevData->DistanceL2)
				{
				if(HAL_GPIO_ReadPin(EPV_ON1_GPIO_Port,EPV_ON1_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR|=0x01;
					else
							DevData->OBR_ERROR&=0x0E;
					if(HAL_GPIO_ReadPin(EPV_ON2_GPIO_Port,EPV_ON2_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR |=0x02;
					else
							DevData->OBR_ERROR &=0x0D;
					if(HAL_GPIO_ReadPin(EPV_ON3_GPIO_Port,EPV_ON3_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR |=0x04;
					else
							DevData->OBR_ERROR &=0x0B;
					if(HAL_GPIO_ReadPin(EPV_ON4_GPIO_Port,EPV_ON4_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR |=0x08;
					else
							DevData->OBR_ERROR &=0x07;
					NozzelAllOn(DevData);
					osDelay(20);
					if(HAL_GPIO_ReadPin(SigKz1_GPIO_Port,SigKz1_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x01;
					if(HAL_GPIO_ReadPin(SigKz2_GPIO_Port,SigKz2_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x02;
					if(HAL_GPIO_ReadPin(SigKz3_GPIO_Port,SigKz3_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x04;
					if(HAL_GPIO_ReadPin(SigKz4_GPIO_Port,SigKz4_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x08;
					osDelay((((uint8_t)DevData->Time_Injection)*100)-20);
					
					DevData->ActualDistance = 0;//обнуляем расстояние пройденное до впрыска
					NozzelAllOff(DevData);
				}
				break;
			case ActivSmazkaL3:
				if(DevData->ActualDistance>=DevData->DistanceL3)
				{
					if(HAL_GPIO_ReadPin(EPV_ON1_GPIO_Port,EPV_ON1_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR|=0x01;
					else
							DevData->OBR_ERROR&=0x0E;
					if(HAL_GPIO_ReadPin(EPV_ON2_GPIO_Port,EPV_ON2_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR |=0x02;
					else
							DevData->OBR_ERROR &=0x0D;
					if(HAL_GPIO_ReadPin(EPV_ON3_GPIO_Port,EPV_ON3_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR |=0x04;
					else
							DevData->OBR_ERROR &=0x0B;
					if(HAL_GPIO_ReadPin(EPV_ON4_GPIO_Port,EPV_ON4_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR |=0x08;
					else
							DevData->OBR_ERROR &=0x07;
					NozzelAllOn(DevData);
					osDelay(20);
					if(HAL_GPIO_ReadPin(SigKz1_GPIO_Port,SigKz1_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x01;
					if(HAL_GPIO_ReadPin(SigKz2_GPIO_Port,SigKz2_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x02;
					if(HAL_GPIO_ReadPin(SigKz3_GPIO_Port,SigKz3_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x04;
					if(HAL_GPIO_ReadPin(SigKz4_GPIO_Port,SigKz4_Pin)==GPIO_PIN_RESET)
						DevData->KZ_ERROR |= 0x08;
					osDelay((((uint8_t)DevData->Time_Injection)*100)-20);
					
					DevData->ActualDistance = 0;//обнуляем расстояние пройденное до впрыска
					NozzelAllOff(DevData);
				}
				break;
			case SelfTest:
				if(DevData->En_Ch)
				{
					for(int i = 0; i<3;i++)
					{
						if(HAL_GPIO_ReadPin(EPV_ON1_GPIO_Port,EPV_ON1_Pin)==GPIO_PIN_RESET)
							DevData->OBR_ERROR|=0x01;
						else
								DevData->OBR_ERROR&=0x0E;
						if(HAL_GPIO_ReadPin(EPV_ON2_GPIO_Port,EPV_ON2_Pin)==GPIO_PIN_RESET)
								DevData->OBR_ERROR |=0x02;
						else
								DevData->OBR_ERROR &=0x0D;
						if(HAL_GPIO_ReadPin(EPV_ON3_GPIO_Port,EPV_ON3_Pin)==GPIO_PIN_RESET)
								DevData->OBR_ERROR |=0x04;
						else
								DevData->OBR_ERROR &=0x0B;
						if(HAL_GPIO_ReadPin(EPV_ON4_GPIO_Port,EPV_ON4_Pin)==GPIO_PIN_RESET)
								DevData->OBR_ERROR |=0x08;
						else
								DevData->OBR_ERROR &=0x07;
						NozzelAllOn(DevData);
						osDelay(20);
						if(HAL_GPIO_ReadPin(SigKz1_GPIO_Port,SigKz1_Pin)==GPIO_PIN_RESET)
							DevData->KZ_ERROR |= 0x01;
						if(HAL_GPIO_ReadPin(SigKz2_GPIO_Port,SigKz2_Pin)==GPIO_PIN_RESET)
							DevData->KZ_ERROR |= 0x02;
						if(HAL_GPIO_ReadPin(SigKz3_GPIO_Port,SigKz3_Pin)==GPIO_PIN_RESET)
							DevData->KZ_ERROR |= 0x04;
						if(HAL_GPIO_ReadPin(SigKz4_GPIO_Port,SigKz4_Pin)==GPIO_PIN_RESET)
							DevData->KZ_ERROR |= 0x08;
							osDelay(SELF_TEST_TIME_ON-20);
							NozzelAllOff(DevData);
							osDelay(SELF_TEST_TIME_OFF);
							
					}
						DevData->Mode = ZapretSmazki;
				}
				else
				{
					NozzelAllOff(DevData);
					CHANNEL_1_POWER_ON
					CHANNEL_1_LED_OK
					osDelay(SELF_TEST_TIME_ON);
					CHANNEL_1_POWER_OFF
					CHANNEL_1_LED_OFF
					osDelay(SELF_TEST_TIME_OFF);
					CHANNEL_1_POWER_ON
					CHANNEL_1_LED_OK
					osDelay(SELF_TEST_TIME_ON);
					CHANNEL_1_POWER_OFF
					CHANNEL_1_LED_OFF
					osDelay(SELF_TEST_TIME_OFF);
					CHANNEL_1_POWER_ON
					CHANNEL_1_LED_OK
					osDelay(SELF_TEST_TIME_ON);
					CHANNEL_1_POWER_OFF
					CHANNEL_1_LED_OFF
					DevData->Mode = ZapretSmazki;
				}
				
				//osDelay(100);
				
				break;
			default:
				
				break;
			
		}
	}
}

void NozzelAllOn (struct DevCondition *DevData)
{
	uint8_t bitState;
	uint8_t ObrBit;
	bitState = (DevData->KZ_ERROR&(1<<0));
	ObrBit = DevData->OBR_ERROR&(1<<0);
	if(DevData->En_Ch&(1<<0))
	{
		if(bitState == 0)
		{
			if(ObrBit ==0)
			{
				CHANNEL_1_POWER_ON
				CHANNEL_1_LED_OK
				DevData->EPV1Count++;
			}
		}

	}
	bitState = (DevData->KZ_ERROR&(1<<1));
	ObrBit = DevData->OBR_ERROR&(1<<1);
	if(DevData->En_Ch&(1<<1))
	{	
		if(bitState == 0)
		{
			if(ObrBit == 0)
			{
				CHANNEL_2_POWER_ON
				CHANNEL_2_LED_OK
				DevData->EPV2Count++;
			}
		}
	}
	ObrBit = DevData->OBR_ERROR&(1<<2);
	bitState = (DevData->KZ_ERROR&(1<<2));
	if(DevData->En_Ch&(1<<2))
	{
		if(bitState == 0)
		{
			if(ObrBit ==0)
			{
				CHANNEL_3_POWER_ON
				CHANNEL_3_LED_OK
				DevData->EPV3Count++;
			}
		}
	}
	ObrBit = DevData->OBR_ERROR&(1<<3);
	bitState = (DevData->KZ_ERROR&(1<<3));
	if(DevData->En_Ch&(1<<3))
	{
		if(bitState == 0)
		{
			if(ObrBit ==0)
			{
				CHANNEL_4_POWER_ON
				CHANNEL_4_LED_OK
				DevData->EPV4Count++;
			}
		}
	}
		/*---
	Сохранение в FRAM
	-----------*/
	WrireToMemUint32(DevData->EPV1Count,FRAM_MEM_ID_EPV1);
	WrireToMemUint32(DevData->EPV2Count,FRAM_MEM_ID_EPV2);
	WrireToMemUint32(DevData->EPV3Count,FRAM_MEM_ID_EPV3);
	WrireToMemUint32(DevData->EPV4Count,FRAM_MEM_ID_EPV4);
	//DevData->Forsunki_Count ++;
	
}
void NozzelAllOff (struct DevCondition *DevData)
{
	CHANNEL_1_POWER_OFF
	CHANNEL_2_POWER_OFF
	CHANNEL_3_POWER_OFF
	CHANNEL_4_POWER_OFF
	CHANNEL_1_LED_OFF
	CHANNEL_2_LED_OFF
	CHANNEL_3_LED_OFF
	CHANNEL_4_LED_OFF
}
void NozzelSingleOn (NumberOfChannel Ch)
{
	switch (Ch){
		case ExitChannel1:
			CHANNEL_1_POWER_ON
			break;
		case ExitChannel2:
			CHANNEL_2_POWER_ON
			break;
		case ExitChannel3:
			CHANNEL_3_POWER_ON
			break;
		case ExitChannel4:
			CHANNEL_4_POWER_ON
			break;
	}
}
void NozzelSingleOff (NumberOfChannel Ch)
{
	switch (Ch){
		case ExitChannel1:
			CHANNEL_1_POWER_OFF
			break;
		case ExitChannel2:
			CHANNEL_2_POWER_OFF
			break;
		case ExitChannel3:
			CHANNEL_3_POWER_OFF
			break;
		case ExitChannel4:
			CHANNEL_4_POWER_OFF
			break;
	}
}
uint8_t OverCurrentSigCheck (void)
{
	uint8_t State = 0x00;
	if(HAL_GPIO_ReadPin(SigKz1_GPIO_Port,SigKz1_Pin)==GPIO_PIN_RESET)
	{
		State |=0x01;
	}
	if(HAL_GPIO_ReadPin(SigKz2_GPIO_Port,SigKz2_Pin)==GPIO_PIN_RESET)
	{
		State |=0x02;
	}
	if(HAL_GPIO_ReadPin(SigKz3_GPIO_Port,SigKz3_Pin)==GPIO_PIN_RESET)
	{
		State |=0x04;
	}
	if(HAL_GPIO_ReadPin(SigKz4_GPIO_Port,SigKz4_Pin)==GPIO_PIN_RESET)
	{
		State |=0x08;
	}
	return State;
}
uint8_t OutSigCheck (void)
{
	uint8_t State = 0x00;
	if(HAL_GPIO_ReadPin(EPV_ON1_GPIO_Port,EPV_ON1_Pin)==GPIO_PIN_SET)
	{
		State |=0x01;
	}
	if(HAL_GPIO_ReadPin(EPV_ON2_GPIO_Port,EPV_ON2_Pin)==GPIO_PIN_SET)
	{
		State |=0x02;
	}
	if(HAL_GPIO_ReadPin(EPV_ON3_GPIO_Port,EPV_ON3_Pin)==GPIO_PIN_SET)
	{
		State |=0x04;
	}
	if(HAL_GPIO_ReadPin(EPV_ON4_GPIO_Port,EPV_ON4_Pin)==GPIO_PIN_SET)
	{
		State |= 0x08;
	}
	return State;
}
void ChannelIndication (void)
{
	/*
	if(HAL_GPIO_ReadPin(KeyOn1_GPIO_Port,KeyOn1_Pin)==GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(ch1_1_GPIO_Port, ch1_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ch1_2_GPIO_Port, ch1_2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(ch1_1_GPIO_Port, ch1_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ch1_2_GPIO_Port, ch1_2_Pin, GPIO_PIN_RESET);
	}
	if(HAL_GPIO_ReadPin(KeyOn2_GPIO_Port,KeyOn2_Pin)==GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(ch2_1_GPIO_Port, ch2_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ch2_2_GPIO_Port, ch2_2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(ch2_1_GPIO_Port, ch2_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ch2_2_GPIO_Port, ch2_2_Pin, GPIO_PIN_RESET);
	}
	if(HAL_GPIO_ReadPin(KeyOn3_GPIO_Port,KeyOn3_Pin)==GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(ch3_1_GPIO_Port, ch3_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ch3_2_GPIO_Port, ch3_2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(ch3_1_GPIO_Port, ch3_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ch3_2_GPIO_Port, ch3_2_Pin, GPIO_PIN_RESET);
	}
	if(HAL_GPIO_ReadPin(KeyOn4_GPIO_Port,KeyOn4_Pin)==GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(ch4_1_GPIO_Port, ch4_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ch4_2_GPIO_Port, ch4_2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(ch4_1_GPIO_Port, ch4_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ch4_2_GPIO_Port, ch4_2_Pin, GPIO_PIN_RESET);
	}
	*/
}
ChannelState GetChannelState (NumberOfChannel Ch)
{
	switch (Ch){
		case ExitChannel1:
			if(HAL_GPIO_ReadPin(EPV_ON1_GPIO_Port,EPV_ON1_Pin)==GPIO_PIN_SET)
				return ChannelOn;
			else
				return ChannelBreak;
			if(HAL_GPIO_ReadPin(SigKz1_GPIO_Port,SigKz1_Pin)==GPIO_PIN_RESET)
				return ChannelShortCircuit;
			break;	
		case ExitChannel2:
			if(HAL_GPIO_ReadPin(EPV_ON2_GPIO_Port,EPV_ON2_Pin)==GPIO_PIN_SET)
				return ChannelOn;
			else
				return ChannelBreak;
			if(HAL_GPIO_ReadPin(SigKz2_GPIO_Port,SigKz2_Pin)==GPIO_PIN_RESET)
				return ChannelShortCircuit;
			break;
		case ExitChannel3:
			if(HAL_GPIO_ReadPin(EPV_ON3_GPIO_Port,EPV_ON3_Pin)==GPIO_PIN_SET)
				return ChannelOn;
			else
				return ChannelBreak;
			if(HAL_GPIO_ReadPin(SigKz3_GPIO_Port,SigKz3_Pin)==GPIO_PIN_RESET)
				return ChannelShortCircuit;
			break;
		case ExitChannel4:
			if(HAL_GPIO_ReadPin(EPV_ON4_GPIO_Port,EPV_ON4_Pin)==GPIO_PIN_SET)
				return ChannelOn;
			else
				return ChannelBreak;
			if(HAL_GPIO_ReadPin(SigKz4_GPIO_Port,SigKz4_Pin)==GPIO_PIN_RESET)
				return ChannelShortCircuit;
			break;
		case NoChannel:
			return NULL;
		break;
	}
}
void NozzelContr(struct DevCondition *DevData,uint8_t *ChannelCondition)
{
	uint8_t Sostoyanie_Port;
	switch(DevData->Mode)
	{
		case ActivSmazkaL1:
			if(DevData->ActualDistance >= DevData->DistanceL1)
			{
				CHANNEL_1_POWER_ON
				CHANNEL_2_POWER_ON
				CHANNEL_3_POWER_ON
				CHANNEL_4_POWER_ON
				osDelay(2);
				if(HAL_GPIO_ReadPin(SigKz1_GPIO_Port,SigKz1_Pin)==GPIO_PIN_SET)
				{
					//ChannelCondition = (uint8_t) 0x01;
				}
				if(HAL_GPIO_ReadPin(SigKz2_GPIO_Port,SigKz2_Pin)==GPIO_PIN_SET)
				{
					
				}
				if(HAL_GPIO_ReadPin(SigKz3_GPIO_Port,SigKz3_Pin)==GPIO_PIN_SET)
				{
					
				}
				if(HAL_GPIO_ReadPin(SigKz4_GPIO_Port,SigKz4_Pin)==GPIO_PIN_SET)
				{
					
				}
			}
			break;
		case ActivSmazkaL2:
			
		break;
		case ActivSmazkaL3:
			
			break;
		case ErrorCAN:
			
			break;
		case ZapretSmazki:
			
			break;
		case ErrorOut:
			
			break;
		case SelfTest:
			
			break;
		/*SelfTest = 0,
	ActivSmazkaL1,
	ActivSmazkaL2,
	ActivSmazkaL3,
	ZapretSmazki,
	ErrorCAN,
	ErrorOut,*/
	}
}

float DistanseCount(uint16_t Speed,int Time)
{
	float Dist;
	if(Speed)
		Dist = (Speed*0.027)*Time;
	return Dist;
}

