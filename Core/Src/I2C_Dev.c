#include "I2C_Dev.h"
//#include "main.h"


void WriteToFram (I2C_HandleTypeDef *hi2c, uint8_t *data, uint8_t Size)
{
		// Write some data to the selected address
		//unsigned char CRC_Byte = Crc8(&data[0],Size);
		//FRAM_MultiByte_Write(hi2c, 0x10000, &data[0], 10);
		
		
}

/*Передаем устройствам по i2c настройки работы*/
void SetInitI2cPer(void)
{
	uint8_t confDataReg;
	confDataReg  = GYRO_I2C_CTRL_REG1_SETTING;
	while( HAL_I2C_Mem_Write(&I2C_PORT,GYRO_I2C_ADRESS,GYRO_I2C_ID_CTRL_REG1,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,GYRO_I2C_TIMEOUT)!= HAL_OK);
	confDataReg  = GYRO_I2C_CTRL_REG2_SETTING;
	while( HAL_I2C_Mem_Write(&I2C_PORT,GYRO_I2C_ADRESS,GYRO_I2C_ID_CTRL_REG2,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,GYRO_I2C_TIMEOUT)!= HAL_OK);
	confDataReg  = GYRO_I2C_CTRL_REG3_SETTING;
	while( HAL_I2C_Mem_Write(&I2C_PORT,GYRO_I2C_ADRESS,GYRO_I2C_ID_CTRL_REG3,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,GYRO_I2C_TIMEOUT)!= HAL_OK);
	confDataReg  = GYRO_I2C_CTRL_REG4_SETTING;
	while( HAL_I2C_Mem_Write(&I2C_PORT,GYRO_I2C_ADRESS,GYRO_I2C_ID_CTRL_REG4,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,GYRO_I2C_TIMEOUT)!= HAL_OK);
	confDataReg  = GYRO_I2C_CTRL_REG5_SETTING;
	while( HAL_I2C_Mem_Write(&I2C_PORT,GYRO_I2C_ADRESS,GYRO_I2C_ID_CTRL_REG5,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,GYRO_I2C_TIMEOUT)!= HAL_OK);
	confDataReg  = GYRO_I2C_TEMP_CNG_REG;
	while( HAL_I2C_Mem_Write(&I2C_PORT,GYRO_I2C_ADRESS,GYRO_I2C_ID_TEMP_CNG_REG,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,GYRO_I2C_TIMEOUT)!= HAL_OK);
	
	confDataReg  = GYRO_I2C_INT1_SET;
	while( HAL_I2C_Mem_Write(&I2C_PORT,GYRO_I2C_ADRESS,GYRO_I2C_ID_INT1_CFG,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,GYRO_I2C_TIMEOUT)!= HAL_OK);

	
	confDataReg = ACSEL_I2C_CTRL_REG1_SETTING;
	while(HAL_I2C_Mem_Write(&I2C_PORT,ACSEL_I2C_ADRESS,ACSEL_I2C_ID_CTRL_REG1,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,ACSEL_I2C_TIMEOUT)!=HAL_OK);
	confDataReg = ACSEL_I2C_CTRL_REG2_SETTING;
	while(HAL_I2C_Mem_Write(&I2C_PORT,ACSEL_I2C_ADRESS,ACSEL_I2C_ID_CTRL_REG2,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,ACSEL_I2C_TIMEOUT)!=HAL_OK);
	confDataReg = ACSEL_I2C_CTRL_REG3_SETTING;
	while(HAL_I2C_Mem_Write(&I2C_PORT,ACSEL_I2C_ADRESS,ACSEL_I2C_ID_CTRL_REG3,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,ACSEL_I2C_TIMEOUT)!=HAL_OK);
	confDataReg = ACSEL_I2C_CTRL_REG4_SETTING;
	while(HAL_I2C_Mem_Write(&I2C_PORT,ACSEL_I2C_ADRESS,ACSEL_I2C_ID_CTRL_REG4,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,ACSEL_I2C_TIMEOUT)!=HAL_OK);
	confDataReg = ACSEL_I2C_CTRL_REG5_SETTING;
	while(HAL_I2C_Mem_Write(&I2C_PORT,ACSEL_I2C_ADRESS,ACSEL_I2C_ID_CTRL_REG5,I2C_MEMADD_SIZE_8BIT,&confDataReg,1,ACSEL_I2C_TIMEOUT)!=HAL_OK);
}

//Функия извлекает значения по всем 3-м координатам uint16_t
/*
struct DataI2CPer GetValueAxis (struct DataI2CPer Str)
{
	uint8_t HiData, LoData;
	if(Str.Adress)
	{
		while(HAL_I2C_GetState(&I2C_PORT)!=HAL_I2C_STATE_READY)
			{
			}
			while(HAL_I2C_Mem_Read(&I2C_PORT,Str.Adress,(GYRO_I2C_ID_OUT_X_L),I2C_MEMADD_SIZE_8BIT, &LoData,1,I2C_TIMEOUT)!=HAL_OK);
			osDelay(1);
			while(HAL_I2C_Mem_Read(&I2C_PORT,Str.Adress,(GYRO_I2C_ID_OUT_X_L+1),I2C_MEMADD_SIZE_8BIT, &HiData,1,I2C_TIMEOUT)!=HAL_OK);
			osDelay(1);
			Str.Xdata = LoData | (HiData<<8);
			while(HAL_I2C_Mem_Read(&I2C_PORT,Str.Adress,(GYRO_I2C_ID_OUT_X_L+2),I2C_MEMADD_SIZE_8BIT, &LoData,1,I2C_TIMEOUT)!=HAL_OK);
			osDelay(1);
			while(HAL_I2C_Mem_Read(&I2C_PORT,Str.Adress,(GYRO_I2C_ID_OUT_X_L+3),I2C_MEMADD_SIZE_8BIT, &HiData,1,I2C_TIMEOUT)!=HAL_OK);
			osDelay(1);
			Str.Ydata = LoData | (HiData<<8);
			while(HAL_I2C_Mem_Read(&I2C_PORT,Str.Adress,(GYRO_I2C_ID_OUT_X_L+4),I2C_MEMADD_SIZE_8BIT, &LoData,1,I2C_TIMEOUT)!=HAL_OK);
			osDelay(1);
			while(HAL_I2C_Mem_Read(&I2C_PORT,Str.Adress,(GYRO_I2C_ID_OUT_X_L+5),I2C_MEMADD_SIZE_8BIT, &HiData,1,I2C_TIMEOUT)!=HAL_OK);
			osDelay(1);
			Str.Zdata = LoData | (HiData<<8);
			return Str;
	}
	else
		return Str;
}
*/

//-------------------------------------------------------------------------------------------------------------------------------------------------
//ИЗВЛЕЧЕНИЕ ДАННЫХ ИЗ КОНКРЕТНОЙ ЯЧЕЙКИ
//-------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t GetValueFromI2C (uint8_t DevAdress,uint8_t IdAdress)
{
	uint8_t Data;
	/*
		while(HAL_I2C_GetState(&I2C_PORT)!=HAL_I2C_STATE_READY)
			{
			}
	*/
			while(HAL_I2C_Mem_Read(&I2C_PORT,DevAdress,(IdAdress),I2C_MEMADD_SIZE_8BIT, &Data,1,I2C_TIMEOUT)!=HAL_OK);
		return Data;
}
/*---------------------------------------*/
//функция состовления буффера из данных по каждой оси гироскопа или акселерометра
void AddValAxisToBuff (uint8_t DevAdress,uint8_t IdAdress, uint16_t *Buff, uint8_t size)
{
	uint8_t LoData,HiData;
	for(uint8_t i = 0;i<size;i++)
	{
			LoData = GetValueFromI2C(DevAdress,IdAdress);
			HiData = GetValueFromI2C(DevAdress,(IdAdress+1));
			Buff[i] = (HiData<<8) | LoData;
	}
}

uint16_t GetSingleAxis (uint8_t DevAdress,uint8_t IdAdress)
{
	uint8_t LoData,HiData;
	uint16_t ret;
		LoData = GetValueFromI2C(DevAdress,IdAdress);
		HiData = GetValueFromI2C(DevAdress,(IdAdress+1));
	ret = (uint16_t)(HiData<<8) | LoData;
	return(ret);
}

/*--------------------------------------------------*/
//Сбор данных по всем трем осям




//-------------------------------------------------------------------------------------------------------------------------------------------------
//функция извлекает буффер из N-го колличества данных по осям со времен

struct DataI2CPer sGetValueAxisBuff(struct DataI2CPer Str, int Size)
{
	uint8_t regData[6];
	if(Size>100) Size=100; 
			while(HAL_I2C_GetState(&I2C_PORT)!=HAL_I2C_STATE_READY);
			for(int i=0;i<Size;i++)
			{
				while(HAL_I2C_Mem_Read(&I2C_PORT,Str.Adress,(GYRO_I2C_ID_OUT_X_L),1, regData,6,10)!=HAL_OK)
					Str.XdataBuff[i] = (int16_t)(regData[0] << 8|regData[1]);
					Str.YdataBuff[i] = (int16_t)(regData[2] << 8|regData[3]);
					Str.ZdataBuff[i] = (int16_t)(regData[4] << 8|regData[5]);
			}
			return Str;
}

/*---------------НЕРАБОТАЕТ---------------------------------------------*/
void vGetAxisValueBuff(uint8_t Adrr,uint8_t AdrrMem,uint16_t *buff,int Size)
{
	uint8_t HiData, LoData;
	while(HAL_I2C_GetState(&I2C_PORT)!=HAL_I2C_STATE_READY){}
			for(int i=0;i<Size;i++)
			{
				while(HAL_I2C_Mem_Read(&I2C_PORT,Adrr,(AdrrMem),I2C_MEMADD_SIZE_8BIT, &LoData,1,I2C_TIMEOUT)!=HAL_OK);
				while(HAL_I2C_Mem_Read(&I2C_PORT,Adrr,(AdrrMem+1),I2C_MEMADD_SIZE_8BIT, &HiData,1,I2C_TIMEOUT)!=HAL_OK);
				buff[i] = LoData | (HiData<<8);
			}
}
/*-----------------------------------------------------------------------*/
float fGetAverageAxis(uint8_t Adrr,uint8_t AdrrMem, uint16_t Size)
{
	uint16_t DataAxisBuff[Size];
	float AverageVal;
	vGetAxisValueBuff(Adrr,AdrrMem,DataAxisBuff,Size);
	for(int i;i<Size;i++)
	{
		AverageVal += DataAxisBuff[i]/Size;
	}
}
float fAverageAxisVal (uint16_t *Buff, int Size)
{
	
	float Xsum;
	uint32_t i32SUM;
	for(int i = 0; i<Size;i++)
	{
		i32SUM+=Buff[i];
	}
	Xsum=(float) i32SUM / Size;
	return Xsum;
}

struct DataI2CPer AverageAxisVal(struct DataI2CPer Str, int Size)
{
	Str.XdataAverage = 0;
	Str.YdataAverage = 0;
	Str.ZdataAverage =	0;
	if(Size>=100) Size = 10;
	float Xsum,Ysum,Zsum;
		for(int j = 0; j<10;j++)
		{
			Xsum+= Str.XdataBuff[j];
			Ysum+=Str.YdataBuff[j];
			Zsum+=Str.ZdataBuff[j];
		} 
		Xsum /= 10;
		Ysum /= 10;
		Zsum /= 10;

	
	Str.XdataAverage = Xsum;
	Str.YdataAverage = Ysum;
	Str.ZdataAverage = Zsum;
	return Str;
}
HAL_StatusTypeDef ReadFromI2C(I2C_HandleTypeDef *i2c,uint16_t DevAdress, uint16_t MemAdress, uint8_t *pData, uint16_t len)
{
	HAL_StatusTypeDef ReturnValue;
	HAL_I2C_Mem_Read(i2c,DevAdress,MemAdress,I2C_MEMADD_SIZE_8BIT, pData,len,I2C_TIMEOUT);
	if(ReturnValue != HAL_OK)
		return ReturnValue;
	
	return ReturnValue;
}
HAL_StatusTypeDef WriteToI2C (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len) 
{
	HAL_StatusTypeDef ReturnValue;
	uint8_t *data;
	HAL_I2C_Mem_Write(hi2c,DevAddress,MemAddress,I2C_MEMADD_SIZE_8BIT,pData,len,ACSEL_I2C_TIMEOUT);
}

void GyroGetValueAxis (uint16_t *Value)
{
	uint8_t HiData, LoData;
	while(HAL_I2C_GetState(&I2C_PORT)!=HAL_I2C_STATE_READY);
	for(int i=0; i < 3; i++)
	{
		while(HAL_I2C_Mem_Read(&I2C_PORT,GYRO_I2C_ADRESS,(GYRO_I2C_ID_OUT_X_L+i),I2C_MEMADD_SIZE_8BIT, &LoData,1,I2C_TIMEOUT)!=HAL_OK);
		while(HAL_I2C_Mem_Read(&I2C_PORT,GYRO_I2C_ADRESS,(GYRO_I2C_ID_OUT_X_L+1+i),I2C_MEMADD_SIZE_8BIT, &HiData,1,I2C_TIMEOUT)!=HAL_OK);
		Value[i] = LoData | (HiData<<8);
	}
		
}
void AcselGetValueAxis(uint16_t *Value)
{
	uint8_t HiData, LoData;
	while(HAL_I2C_GetState(&I2C_PORT)!=HAL_I2C_STATE_READY);
	for(int i=0; i<3; i++)
	{
		while(HAL_I2C_Mem_Read(&I2C_PORT,GYRO_I2C_ADRESS,(GYRO_I2C_ID_OUT_X_L+i),I2C_MEMADD_SIZE_8BIT, &LoData,1,I2C_TIMEOUT)!=HAL_OK);
		while(HAL_I2C_Mem_Read(&I2C_PORT,GYRO_I2C_ADRESS,(GYRO_I2C_ID_OUT_X_L+1+i),I2C_MEMADD_SIZE_8BIT, &HiData,1,I2C_TIMEOUT)!=HAL_OK);
		Value[i] = LoData | (HiData<<8);
	}
}
static int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len)
{
	Reg |= 0x80;
  HAL_I2C_Mem_Write(handle, I3G4250D_I2C_ADD_L, Reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) Bufp, len, 1000);
}
static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
	Reg |= 0x80;
  HAL_I2C_Mem_Read(handle, I3G4250D_I2C_ADD_L, Reg,
                   I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
}

unsigned long kalman_filter (unsigned long ADC_Value)
{
    float x_k1_k1,x_k_k1;
    static float ADC_OLD_Value;
    float Z_k;
    static float P_k1_k1;

    static float Q = KALMAN_KOEF_Q;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
    static float R = KALMAN_KOEF_R; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
    static float Kg = KALMAN_KOEF_Kg;
    static float P_k_k1 = KALMAN_KOEF_P_k_k1;

    float kalman_adc;
    static float kalman_adc_old=0;
    Z_k = ADC_Value;
    x_k1_k1 = kalman_adc_old;

    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1/(P_k_k1 + R);

    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
    P_k1_k1 = (1 - Kg)*P_k_k1;
    P_k_k1 = P_k1_k1;

    ADC_OLD_Value = ADC_Value;
    kalman_adc_old = kalman_adc;

    return kalman_adc;
}

float simpleKalman(float newVal) {
	
	float _err_measure = ERR_MEASURE;  // примерный шум измерений
	float _q = KOEFF_Q; 
  float _kalman_gain, _current_estimate;
  static float _err_estimate = ERR_MEASURE;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}
float ABfilter (float newVal)
{
	float dt = KOEFF_DT;
float sigma_process = SIGMA_PROCESS;
float sigma_noise = SIGMA_NOISE;
	
	static float xk_1, vk_1, a, b;
  static float xk, vk, rk;
  static float xm;
  float lambda = (float)sigma_process * dt * dt / sigma_noise;
  float r = (4 + lambda - (float)sqrt(8 * lambda + lambda * lambda)) / 4;
  a = (float)1 - r * r;
  b = (float)2 * (2 - a) - 4 * (float)sqrt(1 - a);
  xm = newVal;
  xk = xk_1 + ((float) vk_1 * dt );
  vk = vk_1;
  rk = xm - xk;
  xk += (float)a * rk;
  vk += (float)( b * rk ) / dt;
  xk_1 = xk;
  vk_1 = vk;
  return xk_1;
}

/*
void GyroInit(void)
{
	uint8_t WhoAmI;
	stmdev_ctx_t dev_ctx_i3g4250;
	dev_ctx_i3g4250.write_reg = platform_write;
	dev_ctx_i3g4250.read_reg = platform_read;
	dev_ctx_i3g4250.handle = &hi2c2;
	

	i3g4250d_device_id_get(&dev_ctx_i3g4250,&WhoAmI);
	
	i3g4250d_data_rate_set(&dev_ctx_i3g4250,I3G4250D_ODR_100Hz);
}

void AcselInit(void)
{
	stmdev_ctx_t dev_ctx_iis2dh;
	
	dev_ctx_iis2dh.write_reg = platform_write;
	dev_ctx_iis2dh.read_reg = platform_read;
	dev_ctx_iis2dh.handle = &hi2c2;

	iis2dh_data_rate_set(&dev_ctx_iis2dh,IIS2DH_ODR_25Hz);
	iis2dh_full_scale_set(&dev_ctx_iis2dh,IIS2DH_2g);
}
*/
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

void I2CGetMemoryVal(uint8_t DevAdress, uint8_t ID_Adress, uint8_t *buff,uint8_t Size)
{
		while(HAL_I2C_Mem_Read(&hi2c2,DevAdress,ID_Adress,I2C_MEMADD_SIZE_8BIT,buff,Size,I2C_TIMEOUT)!=HAL_OK);
}
void I2CSetMemoryVal(uint8_t DevAdress, uint8_t ID_Adress, uint8_t *buff,uint8_t Size)
{
	while(HAL_I2C_Mem_Write(&hi2c2,DevAdress,ID_Adress,I2C_MEMADD_SIZE_8BIT,buff,Size,I2C_TIMEOUT)!=HAL_OK);
}
int AreaDetected (struct DataI2CPer Gyro, struct DataI2CPer Acsel)
{
	if(Gyro.YdataAverage >= GYRO_TORMOZ_LAVEL_L)
	{
		return 0;
	}
	if(Acsel.Ydata < ACSEL_POVOROT)
	{
		return 1;
	}
}
