/*
 * BMP180.c
 *
 *  Created on: 2021. 12. 20.
 *      Author: sohaenglee
 */

#include "main.h"
#include "BMP180.h"
#include <math.h>

uint16_t oversampling=3;
int32_t  B5;

HAL_StatusTypeDef Init_BMP180_Calib_Param(BMP180_Calib_Param *Calib)
{
	uint8_t 	out_buff[22] = {0,},regVal[2]={0,};
	uint16_t 	CalibData[11];

	regVal[0] = BMP180_AC1;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP180_WRITE_ADDRESS,(uint8_t *)&regVal[0],1,1000)!=HAL_OK);
	while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)BMP180_READ_ADDRESS,(uint8_t *)out_buff,22,1000)!=HAL_OK);

	//	HAL_I2C_Mem_Read(&hi2c1, BMP180_READ_ADDRESS, BMP180_AC1, 1, out_buff, 22, 100);

	for(int i=0;i<11;i++)
	{
		CalibData[i] = out_buff[i*2]<<8 | out_buff[i*2+1];
		if(CalibData[i]==0 || CalibData[i]==0xFFFF) return HAL_ERROR;
	}

	Calib->AC1 = CalibData[0];
	Calib->AC2 = CalibData[1];
	Calib->AC3 = CalibData[2];
	Calib->AC4 = CalibData[3];
	Calib->AC5 = CalibData[4];
	Calib->AC6 = CalibData[5];
	Calib->B1  = CalibData[6];
	Calib->B2  = CalibData[7];
	Calib->MB  = CalibData[8];
	Calib->MC  = CalibData[9];
	Calib->MD  = CalibData[10];

	return HAL_OK;
}

float BMP180_GetTempC()
{
	float 		tempC=0.0;;
	int32_t 	x1,x2,BB5;
	uint8_t 	regVal[2]={0,},tmp[2]={0,};

	regVal[0]=0xF4; regVal[1]=0x2E;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP180_WRITE_ADDRESS,(uint8_t *)regVal,2,1000)!=HAL_OK);
	HAL_Delay(5);
	regVal[0]=0xF6; regVal[1]=0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP180_WRITE_ADDRESS,(uint8_t *)regVal,2,1000)!=HAL_OK);
	while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)BMP180_READ_ADDRESS,(uint8_t *)tmp,2,1000)!=HAL_OK);

	BB5 = (tmp[0]<<8 | tmp[1]);
	x1 = (((int32_t)BB5 - B.AC6) * B.AC5)>>15;
	x2 = (B.MC << 11) / (x1 + B.MD);
	B5 = x1 + x2;
	tempC = ((B5+8) >>4) * 0.1;

	return tempC;
}

int32_t	BMP180_GetPressure()
{
	int32_t 	pressure=0;;
	int32_t 	X1,X2,X3,B3,B6,up,BB5;
	uint32_t	B4,B7=0;
	uint8_t 	regVal[2]={0,},press[3]={0,};

	regVal[0]=0xF4; regVal[1]=0x34+(3<<6);
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP180_WRITE_ADDRESS,(uint8_t *)regVal,2,1000)!=HAL_OK);
	HAL_Delay(40);    // 40 ms wait
	regVal[0]=0xF6; regVal[1]=0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP180_WRITE_ADDRESS,(uint8_t *)regVal,2,1000)!=HAL_OK);
	while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)BMP180_READ_ADDRESS,(uint8_t *)press,3,1000)!=HAL_OK);

	BB5 = ((press[0]<<16) + (press[1]<<8) + press[2]) >> (8-oversampling);

	B6 = B5 - 4000;
	X1 = (B.B2 * (B6 * B6 / 0x1000)) >> 11;
	X2 = B.AC2 * B6 >> 11;
	X3 = X1 + X2;
	B3 = (((B.AC1 * 4 + X3) << oversampling)+2) >> 2;
	X1 = B.AC3 * B6 / 0x2000;
	X2 = (B.B1 * (B6 * B6 / 0x1000)) / 0x10000;
	X3 = ((X1 + X2) + 2) / 0x4;
	B4 = B.AC4 * (uint32_t)(X3 + 32768) / 0x8000;
	B7 = ((uint32_t)BB5-B3) *(50000 >> oversampling);

	if( B7 < 0x80000000)
	{
		pressure = (B7*2) / B4;
	}
	else
	{
		pressure = (B7/B4) * 2;
	}

	X1 = (pressure / 0x100 * (pressure/0x100));
	X1 = (X1 * 3038) / 0x10000;
	X2= (-7357 * pressure)/ 0x10000;
	pressure = pressure + (X1 + X2 + 3791) / 0x10;

	return pressure;
}

float	BMP180_GetAltitude(int32_t press)
{
	float altitude=0;

	altitude = 44330.0f * ( 1.0f - pow((press / 101325.0f),0.1903));

	if((altitude <= -500)||(altitude>=9000u))
	{

	}
	return altitude;
}

















