#include "ModeDes.h"

const unsigned char Crc8Table[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};
 
unsigned char Crc8(unsigned char *pcBlock, unsigned char len)
{
    unsigned char crc = 0xFF;
 
    while (len--)
        crc = Crc8Table[crc ^ *pcBlock++];
 
    return crc;
}




void vSelfTest(void)
{
	
}

void DownloadDataFromMem (struct DevCondition *DevData)
{
	uint8_t DataFromFRAM[40];
	while(FRAM_MultiByte_Selective_Read(&hi2c1,FRAM_MEM_First,DataFromFRAM,30)!=HAL_OK);
	
	/*DevData->EPV1Count = DataFromFRAM[1]|(DataFromFRAM[2]<<7)|(DataFromFRAM[3]<<15)|(DataFromFRAM[4]<<23);
	DevData->EPV2Count = DataFromFRAM[5]|(DataFromFRAM[6]<<7)|(DataFromFRAM[7]<<15)|(DataFromFRAM[8]<<23);
	DevData->EPV3Count = DataFromFRAM[9]|(DataFromFRAM[10]<<7)|(DataFromFRAM[11]<<15)|(DataFromFRAM[12]<<23);
	DevData->EPV4Count = DataFromFRAM[13]|(DataFromFRAM[14]<<7)|(DataFromFRAM[15]<<15)|(DataFromFRAM[16]<<23);
	DevData->NomerBloka = DataFromFRAM[17]|(DataFromFRAM[18]<<7);
	DevData->YearOfManfacture =  DataFromFRAM[20]|(DataFromFRAM[21]<<7);
	DevData->MounthOfManafacture = DataFromFRAM[22];
	DevData->DayOfManafacture = DataFromFRAM[23];
	DevData->TimeOfWork = DataFromFRAM[24]|(DataFromFRAM[25]<<7)|(DataFromFRAM[26]<<15)|(DataFromFRAM[27]<<23);
	DevData->FullDistace = DataFromFRAM[28]|(DataFromFRAM[29]<<7)|(DataFromFRAM[30]<<15)|(DataFromFRAM[31]<<23);
	*/
	//DevData->ProidenoeRasstoyaniye = DataFromFRAM[32]|(DataFromFRAM[33]<<7)|(DataFromFRAM[34]<<15)|(DataFromFRAM[35]<<23);
	
}
void LoadDateToMem(struct DevCondition *DevData)
{
	
}
int WrireToMemUint32(uint32_t NewData,uint8_t Adrr)
{
	uint8_t WriteData[5];
	uint8_t CRC_DATA_FRAM;
	
   uint32_t Data_FRAM=0;
  
	
   WriteData[0] = (uint8_t)NewData;
	WriteData[1] = (uint8_t)(NewData>>8);
	WriteData[2] = (uint8_t)(NewData>>16);
	WriteData[3] = (uint8_t)(NewData>>24);

   WriteData[4] = (uint8_t)Crc8((uint8_t*)&NewData,4);
	FRAM_MultiByte_Write(&hi2c1,Adrr,WriteData,5);
	
	FRAM_MultiByte_Selective_Read(&hi2c1,Adrr,WriteData,5);
	
   Data_FRAM =(uint32_t)WriteData[0];
   Data_FRAM|=((uint32_t)WriteData[1]<<8);
   Data_FRAM|=((uint32_t)WriteData[2]<<16);
   Data_FRAM|=((uint32_t)WriteData[3]<<24);
     CRC_DATA_FRAM = (uint8_t)WriteData[4];
   if((Data_FRAM==NewData) && (Crc8((uint8_t*)&NewData,4)==CRC_DATA_FRAM))
     return 1;
   else return 0;
}
int WrireToMemUint8(uint8_t NewData,uint8_t Adrr)
{
	uint8_t WriteData[2];
	uint8_t CRC_DATA_FRAM;
	
   uint16_t Data_FRAM=0;

   WriteData[0] = (uint8_t)NewData;
   WriteData[1] = (uint8_t)Crc8((uint8_t*)&NewData,2);
	FRAM_MultiByte_Write(&hi2c1,Adrr,WriteData,2);
	
	FRAM_MultiByte_Selective_Read(&hi2c1,Adrr,WriteData,2);
	
   Data_FRAM =(uint8_t)WriteData[0];
     CRC_DATA_FRAM = (uint8_t)WriteData[1];
   if((Data_FRAM==NewData) && (Crc8((uint8_t*)&NewData,2)==CRC_DATA_FRAM))
     return 1;
   else return 0;
}
int WrireToMemUint16(uint16_t NewData,uint8_t Adrr)
{
	uint8_t WriteData[3];
	uint8_t CRC_DATA_FRAM;
	
   uint16_t Data_FRAM=0;
  
	
   WriteData[0] = (uint8_t)NewData;
	WriteData[1] = (uint8_t)(NewData>>8);
   WriteData[2] = (uint8_t)Crc8((uint8_t*)&NewData,2);
	FRAM_MultiByte_Write(&hi2c1,Adrr,WriteData,3);
	
	FRAM_MultiByte_Selective_Read(&hi2c1,Adrr,WriteData,3);
	
   Data_FRAM =(uint32_t)WriteData[0];
   Data_FRAM|=(uint32_t)(WriteData[1]<<8);
     CRC_DATA_FRAM = (uint8_t)WriteData[2];
   if((Data_FRAM==NewData) && (Crc8((uint8_t*)&NewData,2)==CRC_DATA_FRAM))
     return 1;
   else return 0;
}

uint32_t ReadFromMemUint32(uint8_t Adrr)
{
	uint8_t RxBuff[5];
	uint8_t CRC_DATA_FRAM;
	uint32_t Data;
	FRAM_MultiByte_Selective_Read(&hi2c1,Adrr,RxBuff,4);
	 Data = (uint32_t)RxBuff[0];
   Data |=(uint32_t)(RxBuff[1]<<8);
   Data |=(uint32_t)(RxBuff[2]<<16);
   Data |=(uint32_t)(RxBuff[3]<<24);
   return Data;
}
uint16_t ReadFromMemUint16(uint8_t Adrr)
{
	uint8_t RxBuff[2];
	uint32_t Data;
	FRAM_MultiByte_Selective_Read(&hi2c1,Adrr,RxBuff,2);
	 Data = (uint16_t)RxBuff[0];
   Data |=(uint16_t)(RxBuff[1]<<8);
   return Data;
}
uint8_t ReadFromMemUint8(uint8_t Adrr)
{
	uint8_t RxBuff[1];
	uint32_t Data;
	FRAM_MultiByte_Selective_Read(&hi2c1,Adrr,RxBuff,1);
	 Data = (uint8_t)RxBuff[0];
   return Data;
}

/*
Status Write_Odometr (uint64_t New_Odometr,uint16_t Odometr_ADDR){
	
   uint64_t Odometr_FRAM=0;
   uint8_t COUNTER_FRAM=0;
	FRAM_MultiByte_Write(&hi2c1,FRAM_COUNTER,DATA,16)
   WriteByte(Odometr_ADDR,(uint8_t)New_Odometr);
   WriteByte(Odometr_ADDR+1,(uint8_t)(New_Odometr>>8));
   WriteByte(Odometr_ADDR+2,(uint8_t)(New_Odometr>>16));
   WriteByte(Odometr_ADDR+3,(uint8_t)(New_Odometr>>24));
   WriteByte(Odometr_ADDR+4,(uint8_t)(New_Odometr>>32));
   WriteByte(Odometr_ADDR+5,(uint8_t)(New_Odometr>>40));
   WriteByte(Odometr_ADDR+6,(uint8_t)(New_Odometr>>48));
   WriteByte(Odometr_ADDR+7,(uint8_t)(New_Odometr>>56));
   WriteByte(Odometr_ADDR+8,(uint8_t)Crc8((uint8_t*)&New_Odometr,8));
   Odometr_FRAM =(uint64_t)ReadByte(Odometr_ADDR);
   Odometr_FRAM|=((uint64_t)ReadByte(Odometr_ADDR+1)<<8);
   Odometr_FRAM|=((uint64_t)ReadByte(Odometr_ADDR+2)<<16);
   Odometr_FRAM|=((uint64_t)ReadByte(Odometr_ADDR+3)<<24);
   Odometr_FRAM|=((uint64_t)ReadByte(Odometr_ADDR+4)<<32);
   Odometr_FRAM|=((uint64_t)ReadByte(Odometr_ADDR+5)<<40);
   Odometr_FRAM|=((uint64_t)ReadByte(Odometr_ADDR+6)<<48);
   Odometr_FRAM|=((uint64_t)ReadByte(Odometr_ADDR+7)<<56);
   COUNTER_FRAM=(uint8_t)ReadByte(Odometr_ADDR+8);
   if((Odometr_FRAM==New_Odometr) && (Crc8((uint8_t*)&New_Odometr,8)==COUNTER_FRAM))
     return SUCCESS;
   else return ERROR;
  }
*/