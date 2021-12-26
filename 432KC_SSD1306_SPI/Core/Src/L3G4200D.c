/*
 * L3G4200D.c
 *
 *  Created on: Dec 13, 2021
 *      Author: sohaenglee
 */

#include "main.h"
#include "L3G4200D.h"

void 	GY_801_Start()
{
	  HAL_GPIO_WritePin(GPIOB, I2C_VCC_Pin, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, I2C_VCC_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
}

void	Init_L3G4200D()
{
	  uint8_t regVal[2];

	  // FS = 00 (+/- 250 dps full scale)
	  regVal[0] = L3G4200D_CTRL_REG4; regVal[1] = 0x80;  // 00
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)regVal,2,1000)!=HAL_OK);

	  // DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
	  regVal[0] = L3G4200D_CTRL_REG1; regVal[1] = 0x0F;  // FF
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)regVal,2,1000)!=HAL_OK);


	  // set high-pass filter setting
	  regVal[0] = L3G4200D_CTRL_REG2; regVal[1] = 0x00;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)regVal,2,1000)!=HAL_OK);

	  // enable high-pass filter
	  regVal[0] = L3G4200D_CTRL_REG5; regVal[1] = 0x10;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)regVal,2,1000)!=HAL_OK);
}

uint8_t	L3G4200D_WhoAmI()
{
	  uint8_t am = 0x0F;;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&am,1,1000)!=HAL_OK);
	  return am;
}

void	L3G4200D_Read(GYRODATA *G)
{
	  uint8_t am[2];
	  uint8_t rdata[6]={0,0,0,0,0,0};

	  am[0] = L3G4200D_OUT_X_L | (1<<7);    am[1] = 0xFF;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)am,2,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&rdata[0],1,1000)!=HAL_OK);
	  am[0] = L3G4200D_OUT_X_H | (1<<7);    am[1] = 0xFF;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&rdata[1],1,1000)!=HAL_OK);

	  am[0] = L3G4200D_OUT_Y_L | (1<<7);    am[1] = 0xFF;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)am,2,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&rdata[2],1,1000)!=HAL_OK);
	  am[0] = L3G4200D_OUT_Y_H | (1<<7);    am[1] = 0xFF;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&rdata[3],1,1000)!=HAL_OK);

	  am[0] = L3G4200D_OUT_Z_L | (1<<7);    am[1] = 0xFF;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)am,2,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&rdata[4],1,1000)!=HAL_OK);
	  am[0] = L3G4200D_OUT_Z_H | (1<<7);    am[1] = 0xFF;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)L3G4200D,(uint8_t *)&rdata[5],1,1000)!=HAL_OK);

	  G->x = (int16_t)(rdata[0]<<8 | rdata[1]);
	  G->y = (int16_t)(rdata[2]<<8 | rdata[3]);
	  G->z = (int16_t)(rdata[4]<<8 | rdata[5]);
}





















