#ifndef I2C_DEV_H
#define I2C_DEV_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "i3g4250d_reg.h"
#include "iis2dh_reg.h"
#include "../FM24VN10/FM24VN10.h"


extern I2C_HandleTypeDef hi2c2;
//#include "main.h"
//#include "cmsis_os.h"
#define MEM_I2C_PORT &hi2c1
#define MEM_I2C_ADRESS 0x44
#define MEM_I2C_MEM_BANK1 0x01

#define GYRO_TORMOZ_LAVEL_L 0.2
#define GYRO_TORMOZ_LAVEL_H 2.0
#define ACSEL_POVOROT 0.6

#define I2C_TIMEOUT 10
#define GYRO_I2C_ADRESS 0xD0
/*-------------------------------*/
#define GYRO_I2C_ID_Who_Am_I 0x0F
#define GYRO_I2C_ID_OUT_TEMP_L 0x0C
#define GYRO_I2C_ID_OUT_TEMP_H 0x0D
#define GYRO_I2C_ID_STATUS_REG_AUX 0x07
#define GYRO_I2C_ID_OUT_X_L 0x28
#define GYRO_I2C_ID_OUT_X_H 0x29
#define GYRO_I2C_ID_OUT_Y_L 0x2A
#define GYRO_I2C_ID_OUT_Y_H 0x2B
#define GYRO_I2C_ID_OUT_Z_L 0x2C
#define GYRO_I2C_ID_OUT_Z_H 0x2D 
#define GYRO_I2C_ID_CTRL_REG1 0x20
#define GYRO_I2C_ID_FIFO_CTRL_REG 0x2E
#define GYRO_I2C_ID_TEMP_CNG_REG 0x1F
#define GYRO_I2C_TEMP_CNG_REG 0xC0
#define GYRO_I2C_ID_STATUS_REG 0x27//смотреть бит 7-ой
/*----------------------------------*/
#define GYRO_I2C_ID_CTRL_REG1 0x20
#define GYRO_I2C_CTRL_REG1_SETTING 0x5F
#define GYRO_I2C_ID_CTRL_REG2 0x21
#define GYRO_I2C_CTRL_REG2_SETTING 0x2C
#define GYRO_I2C_ID_CTRL_REG3 0x22
#define GYRO_I2C_CTRL_REG3_SETTING 0x00
#define GYRO_I2C_ID_CTRL_REG4 0x23
#define GYRO_I2C_CTRL_REG4_SETTING 0x02
#define GYRO_I2C_ID_CTRL_REG5 0x24
#define GYRO_I2C_CTRL_REG5_SETTING 0x00

#define GYRO_I2C_ID_INT1_CFG 0x30
#define GYRO_I2C_INT1_SET 0x6A
#define GYRO_I2C_ID_INT1_SRC 0x31
//#definr GYRO_I2C_ID_INT1_CFG_SETTING 0x6A

#define GYRO_I2C_ONALLAXIS 0xCF
#define GYRO_I2C_TIMEOUT 10
#define GYRO_I2C_ID_STATUS_REG 0x27
#define GYRO_I2C_ID_OUT_TEMP 0x26
#define GYRO_DATA_SIZE 50

#define ACSEL_I2C_ADRESS 0x32
#define ACSEL_I2C_ID_OUT_X_L 0x28
#define ACSEL_I2C_ID_OUT_X_H 0x29
#define ACSEL_I2C_ID_OUT_Y_L 0x2A
#define ACSEL_I2C_ID_OUT_Y_H 0x2B
#define ACSEL_I2C_ID_OUT_Z_L 0x2C
#define ACSEL_I2C_ID_OUT_Z_H 0x2D
#define ACSEL_I2C_ID_Who_Am_I 0x0F

#define ACSEL_I2C_ID_CTRL_REG1 0x20
#define ACSEL_I2C_CTRL_REG1_SETTING 0xD7
#define ACSEL_I2C_ID_CTRL_REG2 0x21
#define ACSEL_I2C_CTRL_REG2_SETTING 0x04
#define ACSEL_I2C_ID_CTRL_REG3 0x22
#define ACSEL_I2C_CTRL_REG3_SETTING 0x00
#define ACSEL_I2C_ID_CTRL_REG4 0x23
#define ACSEL_I2C_CTRL_REG4_SETTING 0x58
#define ACSEL_I2C_ID_CTRL_REG5 0x24
#define ACSEL_I2C_CTRL_REG5_SETTING 0x00
#define ACSEL_I2C_ID_FIFO_CTRL_REG 0x2E
#define ACSEL_I2C_ONALLAXIS 0xFF
#define ACSEL_I2C_TIMEOUT 10
#define ACSEL_DATA_SIZE 50

#define KALMAN_KOEF_Q 0.0001
#define KALMAN_KOEF_R 0.005
#define KALMAN_KOEF_Kg 0
#define KALMAN_KOEF_P_k_k1 1


#define ERR_MEASURE 1.5 //примерый шум 0.8
#define KOEFF_Q 0.5 //скорость изменения значений val: 0,001 - 1 

#define KOEFF_DT 0.5
#define SIGMA_PROCESS 1.5
#define SIGMA_NOISE 0.9

//Структура хранения данных гироскопа и акселерометра
struct DataI2CPer{
	uint16_t TempData;
	float_t Xdata,Ydata,Zdata;
	float_t XdataAverage,YdataAverage,ZdataAverage;
	float XdataZeroLevel, YdataZeroLevel, ZdataZeroLevel;
	float XdataBuff[100];
	float YdataBuff[100];
	float ZdataBuff[100];
	uint8_t Adress;
	uint8_t BuffSize;
};
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} SensorData;
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;



void SetInitI2cPer(void);
/*--------------------------------------------------------------------------------------------*/
void GyroInit(void);

void I2CGetMemoryVal(uint8_t DevAdress, uint8_t ID_Adress, uint8_t *buff,uint8_t Size);
void I2CSetMemoryVal(uint8_t DevAdress, uint8_t ID_Adress, uint8_t *buff,uint8_t Size);

/*--------------------------------------------------------------------------------------------*/
unsigned char Crc8(unsigned char *pcBlock, unsigned char len);

/*--------------------------------------------------------------------------------------------*/

float simpleKalman(float newVal);
float ABfilter (float newVal);

unsigned long kalman_filter(unsigned long ADC_Value);


double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
//uint8_t WhoAmI;
//void AcselGetValueAxis(uint16_t *Value);

#ifndef I2C_PORT
#define I2C_PORT hi2c2
#endif



#endif
