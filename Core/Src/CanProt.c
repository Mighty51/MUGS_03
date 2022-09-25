#include "CanProt.h"






void IntreputDataFromBis(uint32_t RxIDFrame, uint8_t *RxData, struct ExtData *NewData, struct DevCondition *DevData)
{
	
	int bitState = 0x00;
	
	switch (RxIDFrame){
		case CAN_PROT_IDExt_FORM_BIS_03_CHANNEL_1:
						/*------------1byte------------------*/
						bitState = RxData[0]&(1<<0);
						if(bitState)
							NewData->DirectionСh1 = Backward;
						else
							NewData->DirectionСh1 = Forward;
						/*----------------------------------*/
							bitState = RxData[0]&(1<<1);
						if(bitState)
							NewData->CANErrorCh1 = Error;
						else
							NewData->CANErrorCh1 = NoError;
						/*----------------------------------*/
						bitState = RxData[0]&(1<<2);
						if(bitState)
							NewData->DVHErrorCh1 = Error;
						else
							NewData->DVHErrorCh1 = NoError;
						/*----------------------------------*/
						bitState = RxData[0]&(1<<3);
						if(bitState)
							NewData->ChannelBErrorCh1 = Error;
						else
							NewData->ChannelBErrorCh1 = NoError;
						/*----------------------------------*/
						bitState = RxData[0]&(1<<4);
						if(bitState)
							NewData->ChannelAErrorCh1 = Error;
						else
							NewData->ChannelAErrorCh1 = NoError;
						/*----------------------------------*/
						bitState = RxData[0]&(1<<5);
						if(bitState)
							NewData->ChannelFaultCh1 = Error;
						else
							NewData->ChannelFaultCh1 = NoError;
						/*------------------------------*/
						bitState = RxData[0]&(1<<6);
						if(bitState)
							NewData->ChannelIch1 = ON;
						else
							NewData->ChannelIch1 = OFF;
						/*------------------------------*/
						bitState = RxData[0]&(1<<7);
						if(bitState)
							NewData->ChannelMch1 = ON;
						else
							NewData->ChannelMch1 = OFF;
						/*------------------------------*/
						
						/*------------2byte------------------*/
							NewData->AcselrationCh1 = RxData[1];
						/*------------------------------*/
						
						/*------------3&4byte------------------*/
							NewData->SpeedCh1 = (RxData[3]<<8)|RxData[2];
						/*------------------------------*/
						
						/*------------5byte------------------*/
							NewData->DiametrCh1 = RxData[4];
						/*------------------------------*/
						/*------------6byte------------------*/
							NewData->DiametrCh1 = RxData[5];
						/*------------------------------*/
							NewData->SostCh1 = (RxData[7]<<8)|RxData[6];
						
						DevData->Direction = NewData->DirectionСh1;
						DevData->Speed = (NewData->SpeedCh1)*0.1;
						DevData->Acceleration = NewData->AcselrationCh1*0.01;
						
			break;
		case CAN_PROT_IDExt_FORM_BIS_03_CHANNEL_2:
							bitState = RxData[0]&(1<<0);
							if(bitState)
								NewData->DirectionСh2 = Forward;
							else
								NewData->DirectionСh2 = Backward;
							/*----------------------------------*/
								bitState = RxData[0]&(1<<1);
							if(bitState)
								NewData->CANErrorCh2 = Error;
							else
								NewData->CANErrorCh2 = NoError;
							/*----------------------------------*/
							bitState = RxData[0]&(1<<2);
							if(bitState)
								NewData->DVHErrorCh2 = Error;
							else
								NewData->DVHErrorCh2 = NoError;
							/*----------------------------------*/
							bitState = RxData[0]&(1<<3);
							if(bitState)
								NewData->ChannelBErrorCh2 = Error;
							else
								NewData->ChannelBErrorCh2 = NoError;
							/*----------------------------------*/
							bitState = RxData[0]&(1<<4);
							if(bitState)
								NewData->ChannelAErrorCh2 = Error;
							else
								NewData->ChannelAErrorCh2 = NoError;
							/*----------------------------------*/
							bitState = RxData[0]&(1<<5);
							if(bitState)
								NewData->ChannelFaultCh2 = Error;
							else
								NewData->ChannelFaultCh2 = NoError;
							/*------------------------------*/
							bitState = RxData[0]&(1<<6);
							if(bitState)
								NewData->ChannelIch2 = ON;
							else
								NewData->ChannelIch2 = OFF;
							/*------------------------------*/
							bitState = RxData[0]&(1<<7);
							if(bitState)
								NewData->ChannelMch2 = ON;
							else
								NewData->ChannelMch2 = OFF;
							/*------------------------------*/
							/*------------------------------*/
							
							/*------------2byte------------------*/
								NewData->AcselrationCh2 = RxData[1];
							/*------------------------------*/
							
							/*------------3&4byte------------------*/
								NewData->SpeedCh2 = (RxData[3]<<8)|RxData[2];
							/*------------------------------*/
							
							/*------------5byte------------------*/
								NewData->DiametrCh2 = RxData[4];
							/*------------------------------*/
							/*------------6byte------------------*/
								NewData->DiametrCh2 = RxData[5];
							/*------------------------------*/
								NewData->SostCh2 = (RxData[6]<<8)|RxData[7];
						
							/*------------------------------*/
							
							DevData->Speed = (NewData->SpeedCh2)*0.1;
							DevData->Acceleration = NewData->AcselrationCh2*0.01;
							DevData->Direction = NewData->DirectionСh2;
							
							break;
		/*----------------------------------*/
		case CAN_PROT_IDExt_FORM_BIS_03_COMMAND:
			/*-------------------------------*/
						bitState = RxData[0]&(1<<0);
							if(bitState)
								NewData->cnt = ReciveCommand;
							else
								NewData->cnt = NoCommand;
							/*--------------------------------*/
							bitState = RxData[0]&(1<<1);
							if(bitState)
								NewData->info = ReciveCommand;
							else
								NewData->info = NoCommand;
							/*--------------------------------*/
							bitState = RxData[0]&(1<<2);
							if(bitState)
								NewData->contr = ReciveCommand;
							else
								NewData->contr = NoCommand;
							/*--------------------------------*/
							bitState = RxData[0]&(1<<3);
							if(bitState)
								NewData->T2 = ReciveCommand;
							else
								NewData->T2 = NoCommand;
							/*--------------------------------*/
							bitState = RxData[0]&(1<<4);
							if(bitState)
								NewData->T3 = ReciveCommand;
							else
								NewData->T3 = NoCommand;
							/*--------------------------------*/
							bitState = RxData[0]&(1<<5);
							if(bitState)
								NewData->geroinit = ReciveCommand;
							else
								NewData->geroinit = NoCommand;
							/*--------------------------------*/
							bitState = RxData[0]&(1<<6);
							if(bitState)
								NewData->error_log = ReciveCommand;
							else
								NewData->error_log = NoCommand;
							/*--------------------------------*/
			/*------------------------------*/
			break;
		case CAN_PROT_IDExt_FORM_BIS_03_CONF_1:
			/*-------------------------------*/
					NewData->ParametrV1 = RxData[0];
					NewData->ParametrV2 = RxData[1];
					NewData->ParametrL1 = (RxData[3]<<8)|RxData[2];
					NewData->ParametrL2 = (RxData[5]<<8)|RxData[4];
					NewData->ParametrL3 = (RxData[7]<<8)|RxData[6];
			/*----------массив с внутрениими данными------------------------*/
					DevData->SpeedV1 = NewData->ParametrV1;
					DevData->SpeedV2 = NewData->ParametrV2;
					DevData->DistanceL1 = NewData->ParametrL1;
					DevData->DistanceL2 = NewData->ParametrL2;
					DevData->DistanceL3 = NewData->ParametrL3;
					DevData->Acceleration = NewData->AcselrationCh1*0.01;
					
			/*------------------------------*/
			break;
		case CAN_PROT_IDExt_FORM_BIS_03_CONF_2:
			/*-------------------------------*/
					NewData->ParametrTb = (RxData[1]<<8)|RxData[0];
					NewData->ParametrTp = (RxData[3]<<8)|RxData[2];
					NewData->ParametrAz = (RxData[5]<<8)|RxData[4];
					NewData->ParametrT1 = RxData[6];
					/*-------------------------------*/
						bitState = RxData[7]&(1<<0);
							if(bitState)
								NewData->rsm = ON;
							else
								NewData->rsm = OFF;
							/*--------------------------------*/
							bitState = RxData[7]&(1<<1);
							if(bitState)
								NewData->rzm = ON;
							else
								NewData->rzm = OFF;
							/*--------------------------------*/
							bitState = RxData[7]&(1<<2);
							if(bitState)
								NewData->k1 = ON;
							else
								NewData->k1 = OFF;
							/*--------------------------------*/
							bitState = RxData[7]&(1<<3);
							if(bitState)
								NewData->k2 = ON;
							else
								NewData->k2 = OFF;
							/*--------------------------------*/
							bitState = RxData[7]&(1<<4);
							if(bitState)
								NewData->k3 = ON;
							else
								NewData->k3 = OFF;
							/*--------------------------------*/
							bitState = RxData[7]&(1<<5);
							if(bitState)
								NewData->k4 = ON;
							else
								NewData->k4 = OFF;
							/*--------------------------------*/
							bitState = RxData[7]&(1<<6);
							if(bitState)
								NewData->aof = ON;
							else
								NewData->aof = OFF;
							/*--------------------------------*/
					
					//----------------------------------------------------		
							DevData->Time_Injection = NewData->ParametrTb*0.01;
							DevData->TimeBetweenInjection = NewData->ParametrTp*0.01;
							DevData->PorogAccel = NewData->ParametrAz*0.01;
							DevData->TimeBeforeKontrol = NewData->ParametrT1;
							//DevData->En_Ch = (RxData[7]&(1<<2))|(RxData[7]&(1<<3))|(RxData[7]&(1<<4))|(RxData[7]&(1<<5));
							DevData->En_Ch = NewData->k1|NewData->k2<<1|NewData->k3<<2|NewData->k4<<3;
			/*------------------------------*/
			break;
		case CAN_PROT_IDStd_ACTIVATE:
		
			default:
			
			break;	
	}
}



/*---------------------------------------------------------------------------------------------------------------------------*/
/*------------Формирование буфера на отправку в зависимости от кадра из структуры с данными о состоянии блока----------------*/
/* TxFrameId - ID отправляемоего кадра---------------------------------------------------------------------------------------*/
/* TxDataBuff - массив сообщения (формируется в функции)---------------------------------------------------------------------*/
/* TxDataStruct - структура с даннымо об устройстве--------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------------------*/

void MessageFormateToBis (uint32_t TxFrameId, uint8_t *TxDataBuff, struct ExtData *TxDataStruct, struct DevCondition *DevData)
{
	uint8_t SingleData; 
	switch (TxFrameId){
		/*------------------------------------------------------------------*/
		case CAN_PROT_IDExt_FORM_MUGS_03_CHANNEL_1:
				SingleData = (TxDataStruct->DirectionСh1<<0)|
						(TxDataStruct->CANErrorCh1 <<1)|
						(TxDataStruct->DVHErrorCh1 <<2)|
						(TxDataStruct->ChannelBErrorCh1 <<3)|
						(TxDataStruct->ChannelAErrorCh1 <<4)|
						(TxDataStruct->ChannelFaultCh1 <<5)|
						(TxDataStruct->ChannelIch1 <<6)|
						(TxDataStruct->ChannelMch1 <<7);
				TxDataBuff[0] = SingleData;
				TxDataBuff[1] = TxDataStruct->AcselrationCh1;
				TxDataBuff[2] = (uint8_t) TxDataStruct->SpeedCh1;
				TxDataBuff[3] = (uint8_t) (TxDataStruct->SpeedCh1>>8);
				TxDataBuff[4] = (uint8_t) TxDataStruct->DiametrCh1;
				TxDataBuff[5] = (uint8_t) TxDataStruct->NumberOfGearTeehtCh1;
				TxDataBuff[6] = (uint8_t) TxDataStruct->SostCh1;
				TxDataBuff[7] = (uint8_t) (TxDataStruct->SostCh1>>8);
		break;
		/*------------------------------------------------------------------*/
		case CAN_PROT_IDExt_FORM_MUGS_03_CHANNEL_2:
				SingleData = (TxDataStruct->DirectionСh2<<0)|
						(TxDataStruct->CANErrorCh2 <<1)|
						(TxDataStruct->DVHErrorCh2 <<2)|
						(TxDataStruct->ChannelBErrorCh2 <<3)|
						(TxDataStruct->ChannelAErrorCh2 <<4)|
						(TxDataStruct->ChannelFaultCh2 <<5)|
						(TxDataStruct->ChannelIch2 <<6)|
						(TxDataStruct->ChannelMch2 <<7);
				TxDataBuff[0] = SingleData;
				TxDataBuff[1] = TxDataStruct->AcselrationCh2;
				TxDataBuff[2] = (uint8_t) TxDataStruct->SpeedCh2;
				TxDataBuff[3] = (uint8_t) (TxDataStruct->SpeedCh2>>8);
				TxDataBuff[4] = (uint8_t) TxDataStruct->DiametrCh2;
				TxDataBuff[5] = (uint8_t) TxDataStruct->NumberOfGearTeehtCh2;
				TxDataBuff[6] = (uint8_t) TxDataStruct->SostCh2;
				TxDataBuff[7] = (uint8_t) (TxDataStruct->SostCh2>>8);
			
		break;
		/*------------------------------------------------------------------*/
		case CAN_PROT_IDExt_FORM_MUGS_03_CONF_1:
			TxDataBuff[0] = (uint8_t) TxDataStruct->ParametrV1;
			TxDataBuff[1] = (uint8_t) TxDataStruct->ParametrV2;
			TxDataBuff[2] = (uint8_t) TxDataStruct->ParametrL1;
			TxDataBuff[3] = (uint8_t) (TxDataStruct->ParametrL1>>8);
			TxDataBuff[4] = (uint8_t) TxDataStruct->ParametrL2;
			TxDataBuff[5] = (uint8_t) (TxDataStruct->ParametrL2>>8);
			TxDataBuff[6] = (uint8_t) TxDataStruct->ParametrL3;
			TxDataBuff[7] = (uint8_t) (TxDataStruct->ParametrL3>>8);
		
		break;
		
		/*------------------------------------------------------------------*/
		case CAN_PROT_IDExt_FORM_MUGS_03_CONF_2:
			TxDataBuff[0] = (uint8_t) TxDataStruct->ParametrTb ;
			TxDataBuff[1] = (uint8_t) (TxDataStruct->ParametrTb >>8);
			TxDataBuff[2] = (uint8_t) TxDataStruct->ParametrTp ;
			TxDataBuff[3] = (uint8_t) (TxDataStruct->ParametrTp >>8);
			TxDataBuff[4] = (uint8_t) TxDataStruct->ParametrAz ;
			TxDataBuff[5] = (uint8_t) (TxDataStruct->ParametrAz >>8);
			TxDataBuff[6] = (uint8_t) (TxDataStruct->ParametrT1);
			SingleData = (TxDataStruct->rsm<<0)|
						(TxDataStruct->rzm <<1)|
						(TxDataStruct->k1 <<2)|
						(TxDataStruct->k2 <<3)|
						(TxDataStruct->k3 <<4)|
						(TxDataStruct->k4 <<5)|
						(TxDataStruct->aof <<6);
			TxDataBuff[7] = (uint8_t) SingleData;
		break;
		
		/*------------------------------------------------------------------*/
		case CAN_PROT_IDExt_FORM_MUGS_03_ERRORS:
			SingleData = ((DevData->KZ_ERROR&(1<<0))<<0)|
				((DevData->OBR_ERROR&(1<<0))<<1)|
				((DevData->KZ_ERROR&(1<<1))<<1)|
				((DevData->OBR_ERROR&(1<<1))<<2)|
				((DevData->KZ_ERROR&(1<<2))<<2)|
				((DevData->OBR_ERROR&(1<<2))<<3)|
				((DevData->KZ_ERROR&(1<<3))<<3)|
				((DevData->OBR_ERROR&(1<<3))<<4);

			TxDataBuff[0] = SingleData;
			SingleData = (TxDataStruct->Bak<<0)|
							(TxDataStruct->zs1 <<1)|
							(TxDataStruct->t1 <<2)|
							(TxDataStruct->Pesok <<3)|
							(TxDataStruct->erracc <<4)|
							(TxDataStruct->errgerol <<5)|
							(TxDataStruct->errgeror <<6)|
							(TxDataStruct->errgeroi <<7);
				TxDataBuff[1] = SingleData;
				TxDataBuff[2] = 0x00;
				TxDataBuff[3] = 0x00;
				TxDataBuff[4] = (uint8_t) TxDataStruct->ErrorTime;
				TxDataBuff[5] = (uint8_t) (TxDataStruct->ErrorTime>>8);
				TxDataBuff[6] = (uint8_t) (TxDataStruct->ErrorTime>>16);
				TxDataBuff[7] = (uint8_t) (TxDataStruct->ErrorTime>>24);
				
		break;
		/*------------------------------------------------------------------*/
		case CAN_PROT_IDExt_FORM_MUGS_03_INFO:
				TxDataBuff[0] = (uint8_t) DevData->NomerBloka;
				TxDataBuff[1] = (uint8_t) (DevData->NomerBloka>>8);
				TxDataBuff[2] = (uint8_t) DevData->YearOfManafacture;
				TxDataBuff[3] = (uint8_t) (DevData->YearOfManafacture>>8);
				TxDataBuff[4] = (uint8_t) DevData->MounthOfManafacture;
				TxDataBuff[5] = (uint8_t) DevData->DayOfManafacture;
				TxDataBuff[7] = (uint8_t) TxDataStruct->Ver_po;
				TxDataBuff[7] = (uint8_t) (TxDataStruct->Ver_po >>8);
		break;
		/*------------------------------------------------------------------*/
		case CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV1_2:
			
				TxDataBuff[0] = (uint8_t) DevData->EPV1Count;
				TxDataBuff[1] = (uint8_t) (DevData->EPV1Count>>8);
				TxDataBuff[2] = (uint8_t) (DevData->EPV1Count>>16);
				TxDataBuff[3] = (uint8_t) (DevData->EPV1Count>>24);
				TxDataBuff[4] = (uint8_t) DevData->EPV2Count;
				TxDataBuff[5] = (uint8_t) (DevData->EPV2Count>>8);
				TxDataBuff[6] = (uint8_t) (DevData->EPV2Count>>16);
				TxDataBuff[7] = (uint8_t) (DevData->EPV2Count>>24);

		break;
		/*------------------------------------------------------------------*/
		case CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV3_4:
				TxDataBuff[0] = (uint8_t) DevData->EPV3Count;
				TxDataBuff[1] = (uint8_t) (DevData->EPV3Count>>8);
				TxDataBuff[2] = (uint8_t) (DevData->EPV3Count>>16);
				TxDataBuff[3] = (uint8_t) (DevData->EPV3Count>>24);
				TxDataBuff[4] = (uint8_t) DevData->EPV4Count;
				TxDataBuff[5] = (uint8_t) (DevData->EPV4Count>>8);
				TxDataBuff[6] = (uint8_t) (DevData->EPV4Count>>16);
				TxDataBuff[7] = (uint8_t) (DevData->EPV4Count>>24);
			
		break;
		/*------------------------------------------------------------------*/
		case CAN_PROT_IDExt_FORM_MUGS_03_INFO_TIME:
				TxDataBuff[0] = (uint8_t) DevData->DistanceTraveled;
				TxDataBuff[1] = (uint8_t) (DevData->DistanceTraveled>>8);
				TxDataBuff[2] = (uint8_t) (DevData->DistanceTraveled>>16);
				TxDataBuff[3] = (uint8_t) (DevData->DistanceTraveled>>24);
				TxDataBuff[4] = (uint8_t) DevData->TimeOfWork;
				TxDataBuff[5] = (uint8_t) (DevData->TimeOfWork>>8);
				TxDataBuff[6] = (uint8_t) (DevData->TimeOfWork>>16);
				TxDataBuff[7] = (uint8_t) (DevData->TimeOfWork>24);
		break;
		/*------------------------------------------------------------------*/
	}
}

void CanSendExtMessage (uint32_t TxFrameId, uint8_t *TxDataBuff)
{
	TxHeader.ExtId = TxFrameId;
	TxHeader.DLC = 8;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_EXT;
	
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
	{
		if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxDataBuff,0) !=HAL_OK)
		{
		}			
	}

}

/*---------------------------------------------------------------------------------------*/
/*---Подгружаем значения счетчиков, времени, даты производства из памяти FRAM------------*/
/*---------------------------------------------------------------------------------------*/
/*struct ExtData GetDataFromMem(struct ExtData RxDataStruct)
{
	struct ExtData Data;
	uint8_t DataBuff[8];
	memset(DataBuff,0,8);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_BIS_03_CHANNEL_1,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_BIS_03_CHANNEL_2,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_BIS_03_COMMAND,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_BIS_03_CONF_1,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_BIS_03_CONF_2,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_MUGS_03_CHANNEL_1,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_MUGS_03_CHANNEL_2,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_MUGS_03_CONF_1,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_MUGS_03_CONF_2,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_MUGS_03_ERRORS,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_MUGS_03_INFO,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV1_2,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV3_4,DataBuff);
	Data = IntreputDataFromBis(CAN_PROT_IDExt_FORM_MUGS_03_INFO_TIME,DataBuff);
	return Data;
}
*/
