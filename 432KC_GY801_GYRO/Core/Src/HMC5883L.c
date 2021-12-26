/*
 * HMC5883L.c
 *
 *  Created on: 2021. 12. 15.
 *      Author: sohae
 */
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "HMC5883L.h"

#define RadToDeg 57.29577951f
#define DegToRad 0.017453293f

float 	scale = 0.92,
		Xoffset = 0.0,
		Yoffset = 0.0,
		Zoffset = 0.0,
		angle_offset = 0.0;

uint8_t HMC5883Lmode;

void	Init_HMC5883L()
{
	  uint8_t regVal[2];

	  // Set to 8 samples 15Hz
	  regVal[0] = HMC5883L_CRA; regVal[1] = 0x70;     // 0x70
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)HMC5883L,(uint8_t *)regVal,2,1000)!=HAL_OK);

	  // 1.3 gain LSb / Gauss 1090 (default)
	  regVal[0] = HMC5883L_CRB; regVal[1] = 0x20;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)HMC5883L,(uint8_t *)regVal,2,1000)!=HAL_OK);

	  // Continuous sampling
	  regVal[0] = HMC5883L_MR; regVal[1] = 0x00;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)HMC5883L,(uint8_t *)regVal,2,1000)!=HAL_OK);
}

uint8_t	HMC5883L_WhoAmI()
{
	  uint8_t tmp[4] = {0x0A,0,0,0};
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)HMC5883L,(uint8_t *)&tmp[0],1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)HMC5883L,(uint8_t *)&tmp[1],3,1000)!=HAL_OK);

	  if(tmp[1]=='H' && tmp[2]=='4' && tmp[3]=='3')
	  {
		  return 1;
	  }
	  else return 0;
}

double	HMC5883L_Read()
{
	  uint8_t am;
	  uint8_t rdata[6]={0,0,0,0,0,0};
	  int16_t X,Y,Z;
	  double  x,y,z,bearing,angle;

	  am = HMC5883L_DO_X_H;
	  while(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)HMC5883L,(uint8_t *)&am,1,1000)!=HAL_OK);
	  while(HAL_I2C_Master_Receive (&hi2c1,(uint16_t)HMC5883L,(uint8_t *)rdata,6,1000)!=HAL_OK);

	  X = (int16_t)(rdata[0]<<8 | rdata[1]);
	  x = ((float)X - Xoffset) * scale;
	  Y = (int16_t)(rdata[2]<<8 | rdata[3]);
	  y = ((float)Y - Yoffset) * scale;
	  Z = (int16_t)(rdata[4]<<8 | rdata[5]);
	  z = ((float)Z - Zoffset) * scale;

	  bearing = atan2(x,y) + angle_offset;
	  printf(": %5d %5d %5d",X,Y,Z);
/*
	  if(bearing < 0)
	  {
		  bearing += 360;
	  }
	  bearing += angle_offset;
	  if(bearing < 0)
	  {
		  bearing += 360;
	  }
	  if( bearing > 360)
	  {
		  bearing -= 360;
	  } */
	  angle = bearing;

	  return angle;
}





