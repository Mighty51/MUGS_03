#ifndef CAN_DEV_H
#define CAN_DEV_H

/*Описание протокала обмена по CAN БИС-03 и МУГС-02 */

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_def.h"
#include "main.h"

#define CAN_PROT_IDExt_FORM_BIS_03_CHANNEL_1 0x3710040
#define CAN_PROT_IDExt_FORM_BIS_03_CHANNEL_2 0x3710041
#define CAN_PROT_IDExt_FORM_BIS_03_COMMAND 0x31B0042
#define CAN_PROT_IDExt_FORM_BIS_03_CONF_1 0x31B0043
#define CAN_PROT_IDExt_FORM_BIS_03_CONF_2 0x31B0044
#define CAN_PROT_IDStd_ACTIVATE 0x0135

#define CAN_PROT_IDExt_FORM_MUGS_03_CHANNEL_1 0x3B10040
#define CAN_PROT_IDExt_FORM_MUGS_03_CHANNEL_2 0x3B10041
#define CAN_PROT_IDExt_FORM_MUGS_03_CONF_1 0x3B10043
#define CAN_PROT_IDExt_FORM_MUGS_03_CONF_2 0x3B10044
#define CAN_PROT_IDExt_FORM_MUGS_03_ERRORS 0x3B10045
#define CAN_PROT_IDExt_FORM_MUGS_03_INFO 0x3B10046
#define CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV1_2 0x3B10047
#define CAN_PROT_IDExt_FORM_MUGS_03_INFO_EPV3_4 0x3B10048
#define CAN_PROT_IDExt_FORM_MUGS_03_INFO_TIME 0x3B10049

#define CAN_FILTER_ID_0 0x31B0040
#define CAN_FILTER_MASK_0 0x1FFFFFF8
#define CAN_FILTER_ID_1 0x3710040
#define CAN_FILTER_MASK_1 0x1FFFFFFC
#define CAN_FILTER_ID_2 0x0135
#define CAN_FILTER_MASK_2 0x7FF

extern CAN_HandleTypeDef hcan1;
extern CAN_RxHeaderTypeDef RxHeader;
extern CAN_TxHeaderTypeDef TxHeader;


struct DevCondition;

typedef enum
{
	Forward = 0,
	Backward
}DirectOfTrawel;

typedef enum
{
	NoError = 0,
	Error
}Error_NoError;

typedef enum
{
	NoCommand = 0,
	ReciveCommand
}RxCommand;

typedef enum
{
	OFF = 0,
	ON
}On_Off;

struct ShortData {
	
	/*--DATA FROM FRAMES 0X31B0040 & 0X31B0041--*/
	DirectOfTrawel DirectionСh1;
	DirectOfTrawel DirectionСh2;
	Error_NoError CANErrorCh1;
	Error_NoError CANErrorCh2;
	Error_NoError DVHErrorCh1;
	Error_NoError DVHErrorCh2;
	Error_NoError ChannelBErrorCh1;
	Error_NoError ChannelBErrorCh2;
	Error_NoError ChannelAErrorCh1;
	Error_NoError ChannelAErrorCh2;
	Error_NoError ChannelFaultCh1;
	Error_NoError ChannelFaultCh2;
	On_Off ChannelIch1;//КАНАЛ
	On_Off ChannelIch2;//КАНАЛ
	On_Off ChannelMch1;//КАНАЛ
	On_Off ChannelMch2;//КАНАЛ
	uint8_t AcselrationCh1;//целое значение со знаком, дискрет 0,01м/с2
	uint8_t AcselrationCh2;//целое значение со знаком, дискрет 0,01м/с2
	uint16_t SpeedCh1;//целое значение, дискрет 0,1км/ч
	uint16_t SpeedCh2;//целое значение, дискрет 0,1км/ч
	uint8_t DiametrCh1;//d = D-650,mm Целое значение, дискрет - 1мм
	uint8_t DiametrCh2;//d = D-650,mm Целое значение, дискрет - 1мм
	uint8_t NumberOfGearTeehtCh1;//z = Z/2, Целое значение, дискрет - 1
	uint8_t NumberOfGearTeehtCh2;//z = Z/2, Целое значение, дискрет - 1
	uint16_t SostCh1;//Sост, м целое дискретное
	uint16_t SostCh2;//Sост, м целое дискретное
	/*-----------------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0042---*/
	RxCommand cnt;//передача состояния счетчиков
	RxCommand info;//передача состояния о блоке
	RxCommand contr; //произвести контроль
	RxCommand T2;//тормоз
	RxCommand T3;//песок
	RxCommand geroinit;//Произвести инициализацию гироскопа 
	RxCommand error_log;// передать лог ошибок модуля
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0043---*/
	uint8_t ParametrV1;//параметр V1 Целоеб дискрет 1 км/ч
	uint8_t ParametrV2;//параметр V2 Целоеб дискрет 1 км/ч
	uint16_t ParametrL1;//параметр L1 Целоеб дискрет 1 м
	uint16_t ParametrL2;//параметр L2 Целоеб дискрет 1 м
	uint16_t ParametrL3;//параметр L3 Целоеб дискрет 1 м
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0044---*/
	uint16_t ParametrTb;// Время впрыска ЭПВ, Целое дискретное 0,001с
	uint16_t ParametrTp;// Минимальное время между впрысками ЭПВ Целое дискретное 0,001с
	uint16_t ParametrAz;//Пороговое ускорение Целое дискретное 0,001с
	uint8_t ParametrT1;//Праметр Т1 - задержка перед началом процедуры "контроль" Целое дискретное - 1с
	On_Off rsm; //Режим смазки 1-Второй, 0-первый
	On_Off rzm; //режим запрета смазки 
	On_Off k1; //Включить 1-й канал
	On_Off k2; //Включить 2-й канал
	On_Off k3; //Включить 3-й канал
	On_Off k4; //Включить 4-й канал
	On_Off aof; //Акселерометр МПИС-а
	/*------------------------------*/
	

};

struct ExtData{
	
	/*--DATA FROM FRAMES 0X31B0040 & 0X31B0041--*/
	DirectOfTrawel DirectionСh1;
	DirectOfTrawel DirectionСh2;
	Error_NoError CANErrorCh1;
	Error_NoError CANErrorCh2;
	Error_NoError DVHErrorCh1;
	Error_NoError DVHErrorCh2;
	Error_NoError ChannelBErrorCh1;
	Error_NoError ChannelBErrorCh2;
	Error_NoError ChannelAErrorCh1;
	Error_NoError ChannelAErrorCh2;
	Error_NoError ChannelFaultCh1;
	Error_NoError ChannelFaultCh2;
	On_Off ChannelIch1;//КАНАЛ
	On_Off ChannelIch2;//КАНАЛ
	On_Off ChannelMch1;//КАНАЛ
	On_Off ChannelMch2;//КАНАЛ
	uint8_t AcselrationCh1;//целое значение со знаком, дискрет 0,01м/с2
	uint8_t AcselrationCh2;//целое значение со знаком, дискрет 0,01м/с2
	uint16_t SpeedCh1;//целое значение, дискрет 0,1км/ч
	uint16_t SpeedCh2;//целое значение, дискрет 0,1км/ч
	uint8_t DiametrCh1;//d = D-650,mm Целое значение, дискрет - 1мм
	uint8_t DiametrCh2;//d = D-650,mm Целое значение, дискрет - 1мм
	uint8_t NumberOfGearTeehtCh1;//z = Z/2, Целое значение, дискрет - 1
	uint8_t NumberOfGearTeehtCh2;//z = Z/2, Целое значение, дискрет - 1
	uint16_t SostCh1;//Sост, м целое дискретное
	uint16_t SostCh2;//Sост, м целое дискретное
	/*-----------------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0042---*/
	RxCommand cnt;//передача состояния счетчиков
	RxCommand info;//передача состояния о блоке
	RxCommand contr; //произвести контроль
	RxCommand T2;//тормоз
	RxCommand T3;//песок
	RxCommand geroinit;//Произвести инициализацию гироскопа 
	RxCommand error_log;// передать лог ошибок модуля
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0043---*/
	uint8_t ParametrV1;//параметр V1 Целоеб дискрет 1 км/ч
	uint8_t ParametrV2;//параметр V2 Целоеб дискрет 1 км/ч
	uint16_t ParametrL1;//параметр L1 Целоеб дискрет 1 м
	uint16_t ParametrL2;//параметр L2 Целоеб дискрет 1 м
	uint16_t ParametrL3;//параметр L3 Целоеб дискрет 1 м
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0044---*/
	uint16_t ParametrTb;// Время впрыска ЭПВ, Целое дискретное 0,001с
	uint16_t ParametrTp;// Минимальное время между впрысками ЭПВ Целое дискретное 0,001с
	uint16_t ParametrAz;//Пороговое ускорение Целое дискретное 0,001с
	uint8_t ParametrT1;//Праметр Т1 - задержка перед началом процедуры "контроль" Целое дискретное - 1с
	On_Off rsm; //Режим смазки 1-Второй, 0-первый
	On_Off rzm; //режим запрета смазки 
	On_Off k1; //Включить 1-й канал
	On_Off k2; //Включить 2-й канал
	On_Off k3; //Включить 3-й канал
	On_Off k4; //Включить 4-й канал
	On_Off aof; //Акселерометр МПИС-а
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10045---*/
	Error_NoError ekz1;//КЗ 1-го ЭПВ
	Error_NoError eobr1;//Обрыв 1-го ЭПВ
	Error_NoError ekz2;//КЗ 2-го ЭПВ
	Error_NoError eobr2;//Обрыв 2-го ЭПВ
	Error_NoError ekz3;//КЗ 3-го ЭПВ
	Error_NoError eobr3;//Обрыв 3-го ЭПВ
	Error_NoError ekz4;//КЗ 4-го ЭПВ
	Error_NoError eobr4;//Обрыв 4-го ЭПВ
	
	RxCommand Bak;//Пришел сигнал "БАК"
	RxCommand zs1;//Пришел сигнал "ЗС"
	RxCommand t1;//Пришел сигнал "Т2"
	RxCommand Pesok;//пришел сигнал "Песок"
	RxCommand erracc;//ошибка акселерометра
	RxCommand errgerol;//ошибка теста гироскопа(лево)
	RxCommand errgeror;//ошибка теста гироскопа(право)
	RxCommand errgeroi;//ошибка теста гироскопа(внутр)
	
	uint32_t ErrorTime;//время возникновения ошибки
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10046---*/
	uint16_t Nbl;//номер блока
	uint16_t Year;//год производства
	uint8_t Month;//месяц производства
	uint8_t Day;//день производства
	uint16_t Ver_po;//версия прошивки
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10047---*/
	uint32_t cnt_1;//счетчик впрысков ЭПВ1
	uint32_t cnt_2;//счетчик впрысков ЭПВ2
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10048---*/
	uint32_t cnt_3;//счетчик впрысков ЭПВ3
	uint32_t cnt_4;//счетчик впрысков ЭПВ4
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10049---*/
	uint32_t length;//пройденный путь в км. Целое дискрет - 1км
	uint32_t TimeToFailure;//Счетчик наработки на отказ. Целое дискрет - 1ч
	
};
/*----------------Функции-обработки-принятых-сообщений---------------------------------*/
//struct ExtData IntreputDataFromBis(uint32_t RxIDFrame, uint8_t *RxData);
void IntreputDataFromBis(uint32_t RxIDFrame, uint8_t *RxData, struct ExtData *NewData, struct DevCondition *DevData);
void MessageFormateToBis (uint32_t TxFrameId, uint8_t *TxDataBuff, struct ExtData *TxDataStruct, struct DevCondition *DevData);
void CanSendExtMessage (uint32_t TxFrameId, uint8_t *TxDataBuff);
struct ExtData GetDataFromMem(struct ExtData RxDataStruct);
void StructNull (struct ExtData *RxDataStruct);

void DownloadDataFromMem (struct DevCondition *Data);//загружаем данные о блоке из памяти
void LoadDataToMem (struct DevCondition *Data);//загружаем данные о блоке из памяти


#endif
