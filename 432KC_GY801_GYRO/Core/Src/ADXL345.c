/*
 * ADXL345.c
 *
 *  Created on: 2021. 12. 15.
 *      Author: sohaenglee
 */

#include "main.h"
#include "ADXL345.h"

void	Init_ADXL345()
{
	  uint8_t regVal[2];

	  // Measurement mode
	  regVal[0] = ADXL345_POWER_CTL; regVal[1] = ADXL345_MEASURE;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)regVal,2,1000)!=HAL_OK);

	  // Data ready
	  regVal[0] = ADXL345_INT_ENABLE; regVal[1] = 0x80;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)regVal,2,1000)!=HAL_OK);


	  // Full resolution +-16g
	  regVal[0] = ADXL345_DATA_FORMAT; regVal[1] = ADXL345_BW_RATE_200HZ;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)regVal,2,1000)!=HAL_OK);
}

uint8_t	ADXL345_WhoAmI()
{
	  uint8_t am = ADXL345_DEVID;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)ADXL345,(uint8_t *)&am,1,1000)!=HAL_OK);
	  return am;
}

void	ADXL345_Read(ACCDATA *G)
{
	  uint8_t am;
	  uint8_t rdata[6]={0,0,0,0,0,0};

	  am = ADXL345_DATAX0;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)ADXL345,(uint8_t *)&rdata,6,1000)!=HAL_OK);
	  /*
	  am = ADXL345_DATAX1;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)ADXL345,(uint8_t *)&rdata[1],1,1000)!=HAL_OK);
	  am = ADXL345_DATAY0;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)ADXL345,(uint8_t *)&rdata[2],1,1000)!=HAL_OK);
	  am = ADXL345_DATAY1;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)ADXL345,(uint8_t *)&rdata[3],1,1000)!=HAL_OK);
	  am = ADXL345_DATAZ0;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)ADXL345,(uint8_t *)&rdata[4],1,1000)!=HAL_OK);
	  am = ADXL345_DATAZ1;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)ADXL345,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)ADXL345,(uint8_t *)&rdata[5],1,1000)!=HAL_OK);
	  */
	  G->x = (int16_t)(rdata[0]<<8 | rdata[1]);
	  G->y = (int16_t)(rdata[2]<<8 | rdata[3]);
	  G->z = (int16_t)(rdata[4]<<8 | rdata[5]);
}












