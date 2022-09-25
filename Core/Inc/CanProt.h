#ifndef CAN_DEV_H
#define CAN_DEV_H

/*�������� ��������� ������ �� CAN ���-03 � ����-02 */

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
	DirectOfTrawel Direction�h1;
	DirectOfTrawel Direction�h2;
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
	On_Off ChannelIch1;//�����
	On_Off ChannelIch2;//�����
	On_Off ChannelMch1;//�����
	On_Off ChannelMch2;//�����
	uint8_t AcselrationCh1;//����� �������� �� ������, ������� 0,01�/�2
	uint8_t AcselrationCh2;//����� �������� �� ������, ������� 0,01�/�2
	uint16_t SpeedCh1;//����� ��������, ������� 0,1��/�
	uint16_t SpeedCh2;//����� ��������, ������� 0,1��/�
	uint8_t DiametrCh1;//d = D-650,mm ����� ��������, ������� - 1��
	uint8_t DiametrCh2;//d = D-650,mm ����� ��������, ������� - 1��
	uint8_t NumberOfGearTeehtCh1;//z = Z/2, ����� ��������, ������� - 1
	uint8_t NumberOfGearTeehtCh2;//z = Z/2, ����� ��������, ������� - 1
	uint16_t SostCh1;//S���, � ����� ����������
	uint16_t SostCh2;//S���, � ����� ����������
	/*-----------------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0042---*/
	RxCommand cnt;//�������� ��������� ���������
	RxCommand info;//�������� ��������� � �����
	RxCommand contr; //���������� ��������
	RxCommand T2;//������
	RxCommand T3;//�����
	RxCommand geroinit;//���������� ������������� ��������� 
	RxCommand error_log;// �������� ��� ������ ������
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0043---*/
	uint8_t ParametrV1;//�������� V1 ������ ������� 1 ��/�
	uint8_t ParametrV2;//�������� V2 ������ ������� 1 ��/�
	uint16_t ParametrL1;//�������� L1 ������ ������� 1 �
	uint16_t ParametrL2;//�������� L2 ������ ������� 1 �
	uint16_t ParametrL3;//�������� L3 ������ ������� 1 �
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0044---*/
	uint16_t ParametrTb;// ����� ������� ���, ����� ���������� 0,001�
	uint16_t ParametrTp;// ����������� ����� ����� ��������� ��� ����� ���������� 0,001�
	uint16_t ParametrAz;//��������� ��������� ����� ���������� 0,001�
	uint8_t ParametrT1;//������� �1 - �������� ����� ������� ��������� "��������" ����� ���������� - 1�
	On_Off rsm; //����� ������ 1-������, 0-������
	On_Off rzm; //����� ������� ������ 
	On_Off k1; //�������� 1-� �����
	On_Off k2; //�������� 2-� �����
	On_Off k3; //�������� 3-� �����
	On_Off k4; //�������� 4-� �����
	On_Off aof; //������������ ����-�
	/*------------------------------*/
	

};

struct ExtData{
	
	/*--DATA FROM FRAMES 0X31B0040 & 0X31B0041--*/
	DirectOfTrawel Direction�h1;
	DirectOfTrawel Direction�h2;
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
	On_Off ChannelIch1;//�����
	On_Off ChannelIch2;//�����
	On_Off ChannelMch1;//�����
	On_Off ChannelMch2;//�����
	uint8_t AcselrationCh1;//����� �������� �� ������, ������� 0,01�/�2
	uint8_t AcselrationCh2;//����� �������� �� ������, ������� 0,01�/�2
	uint16_t SpeedCh1;//����� ��������, ������� 0,1��/�
	uint16_t SpeedCh2;//����� ��������, ������� 0,1��/�
	uint8_t DiametrCh1;//d = D-650,mm ����� ��������, ������� - 1��
	uint8_t DiametrCh2;//d = D-650,mm ����� ��������, ������� - 1��
	uint8_t NumberOfGearTeehtCh1;//z = Z/2, ����� ��������, ������� - 1
	uint8_t NumberOfGearTeehtCh2;//z = Z/2, ����� ��������, ������� - 1
	uint16_t SostCh1;//S���, � ����� ����������
	uint16_t SostCh2;//S���, � ����� ����������
	/*-----------------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0042---*/
	RxCommand cnt;//�������� ��������� ���������
	RxCommand info;//�������� ��������� � �����
	RxCommand contr; //���������� ��������
	RxCommand T2;//������
	RxCommand T3;//�����
	RxCommand geroinit;//���������� ������������� ��������� 
	RxCommand error_log;// �������� ��� ������ ������
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0043---*/
	uint8_t ParametrV1;//�������� V1 ������ ������� 1 ��/�
	uint8_t ParametrV2;//�������� V2 ������ ������� 1 ��/�
	uint16_t ParametrL1;//�������� L1 ������ ������� 1 �
	uint16_t ParametrL2;//�������� L2 ������ ������� 1 �
	uint16_t ParametrL3;//�������� L3 ������ ������� 1 �
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X31B0044---*/
	uint16_t ParametrTb;// ����� ������� ���, ����� ���������� 0,001�
	uint16_t ParametrTp;// ����������� ����� ����� ��������� ��� ����� ���������� 0,001�
	uint16_t ParametrAz;//��������� ��������� ����� ���������� 0,001�
	uint8_t ParametrT1;//������� �1 - �������� ����� ������� ��������� "��������" ����� ���������� - 1�
	On_Off rsm; //����� ������ 1-������, 0-������
	On_Off rzm; //����� ������� ������ 
	On_Off k1; //�������� 1-� �����
	On_Off k2; //�������� 2-� �����
	On_Off k3; //�������� 3-� �����
	On_Off k4; //�������� 4-� �����
	On_Off aof; //������������ ����-�
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10045---*/
	Error_NoError ekz1;//�� 1-�� ���
	Error_NoError eobr1;//����� 1-�� ���
	Error_NoError ekz2;//�� 2-�� ���
	Error_NoError eobr2;//����� 2-�� ���
	Error_NoError ekz3;//�� 3-�� ���
	Error_NoError eobr3;//����� 3-�� ���
	Error_NoError ekz4;//�� 4-�� ���
	Error_NoError eobr4;//����� 4-�� ���
	
	RxCommand Bak;//������ ������ "���"
	RxCommand zs1;//������ ������ "��"
	RxCommand t1;//������ ������ "�2"
	RxCommand Pesok;//������ ������ "�����"
	RxCommand erracc;//������ �������������
	RxCommand errgerol;//������ ����� ���������(����)
	RxCommand errgeror;//������ ����� ���������(�����)
	RxCommand errgeroi;//������ ����� ���������(�����)
	
	uint32_t ErrorTime;//����� ������������� ������
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10046---*/
	uint16_t Nbl;//����� �����
	uint16_t Year;//��� ������������
	uint8_t Month;//����� ������������
	uint8_t Day;//���� ������������
	uint16_t Ver_po;//������ ��������
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10047---*/
	uint32_t cnt_1;//������� �������� ���1
	uint32_t cnt_2;//������� �������� ���2
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10048---*/
	uint32_t cnt_3;//������� �������� ���3
	uint32_t cnt_4;//������� �������� ���4
	/*------------------------------*/
	
	/*--DATA FROM FRAME 0X3B10049---*/
	uint32_t length;//���������� ���� � ��. ����� ������� - 1��
	uint32_t TimeToFailure;//������� ��������� �� �����. ����� ������� - 1�
	
};
/*----------------�������-���������-��������-���������---------------------------------*/
//struct ExtData IntreputDataFromBis(uint32_t RxIDFrame, uint8_t *RxData);
void IntreputDataFromBis(uint32_t RxIDFrame, uint8_t *RxData, struct ExtData *NewData, struct DevCondition *DevData);
void MessageFormateToBis (uint32_t TxFrameId, uint8_t *TxDataBuff, struct ExtData *TxDataStruct, struct DevCondition *DevData);
void CanSendExtMessage (uint32_t TxFrameId, uint8_t *TxDataBuff);
struct ExtData GetDataFromMem(struct ExtData RxDataStruct);
void StructNull (struct ExtData *RxDataStruct);

void DownloadDataFromMem (struct DevCondition *Data);//��������� ������ � ����� �� ������
void LoadDataToMem (struct DevCondition *Data);//��������� ������ � ����� �� ������


#endif
