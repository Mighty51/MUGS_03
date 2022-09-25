#ifndef __MODEDES_H
#define __MODEDES_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../FM24VN10/FM24VN10.h"
#include "main.h"

#define FRAM_MEM_First 0x00001F
#define FRAM_MEM_ID_EPV1 0x000025
#define FRAM_MEM_ID_EPV2 0x00002B
#define FRAM_MEM_ID_EPV3 0x000031
#define FRAM_MEM_ID_EPV4 0x000037
#define FRAM_MEM_ID_DISTANCE 0x00003D
#define FRAM_MEM_ID_TIME 0x000043
#define FRAM_MEM_ID_TIME_MIN 0x000049
#define FRAM_MEM_ID_DATA_DAY 0x00004F
#define FRAM_MEM_ID_DATA_YEAR  0x000055
#define FRAM_MEM_ID_DATA_MOUNTH  0x00005B
#define FRAM_MEM_ID_SER_NUM  0x000061



struct DevCondition;
extern I2C_HandleTypeDef hi2c1;
void vSelfTest(void);//обработка проверки самотетст
void DownloadDataFromMem (struct DevCondition *DevData);
void LoadDataToMem(struct DevCondition *DevData);
void LoadCounts(struct DevCondition *DevData);
void LoadDateToMem(struct DevCondition *DevData);
unsigned char Crc8(unsigned char *pcBlock, unsigned char len);

uint8_t ReadFromMemUint8(uint8_t Adrr);
uint16_t ReadFromMemUint16(uint8_t Adrr);
uint32_t ReadFromMemUint32(uint8_t Adrr);

int WrireToMemUint8(uint8_t NewData,uint8_t Adrr);
int WrireToMemUint16(uint16_t NewData,uint8_t Adrr);
int WrireToMemUint32(uint32_t NewData,uint8_t Adrr);

#endif

