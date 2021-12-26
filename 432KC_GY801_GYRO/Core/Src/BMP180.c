/*
 * BMP180.c
 *
 *  Created on: 2021. 12. 20.
 *      Author: sohaenglee
 */

#include "main.h"
#include "BMP180.h"

uint16_t oversampling=3;

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
	int32_t 	buf,x1,x2;
	uint8_t 	regVal[2]={0,},tmp[2]={0,};

	regVal[0]=0xF4; regVal[1]=0x2E;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP180_WRITE_ADDRESS,(uint8_t *)regVal,2,1000)!=HAL_OK);
	HAL_Delay(5);
	regVal[0]=0xF6; regVal[1]=0x00;
	while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)BMP180_WRITE_ADDRESS,(uint8_t *)regVal,2,1000)!=HAL_OK);
	while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)BMP180_READ_ADDRESS,(uint8_t *)tmp,2,1000)!=HAL_OK);

	printf("%d,%d",tmp[0],tmp[1]);

	buf = (tmp[0]<<8 | tmp[1]);
	x1 = ((buf - B.AC6) * B.AC5)>>15;
	x2 = (B.MC << 11) / (x1 + B.MD);
	tempC = ((x1+x2+8) >>4) * 0.1;

	return tempC;
}




















