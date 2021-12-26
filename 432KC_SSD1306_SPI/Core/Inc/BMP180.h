/*
 * BMP180.h
 *
 *  Created on: 2021. 12. 20.
 *      Author: sohaenglee
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

extern I2C_HandleTypeDef hi2c1;

#define BMP180_WRITE_ADDRESS		0x77<<1
#define BMP180_READ_ADDRESS			(BMP180_WRITE_ADDRESS)+1

enum BMP180_regAddr
{
	// HMC5883L (Magnetometer) constants
	BMP180_ADDRESS			= 0x77,
	BMP180_AC1				= 0xAA,
	BMP180_AC2				= 0xAC,
	BMP180_AC3				= 0xAE,
	BMP180_AC4				= 0xB0,
	BMP180_AC5				= 0xB2,
	BMP180_AC6				= 0xB4,
	BMP180_B1				= 0xB6 ,
	BMP180_B2				= 0xB8 ,
	BMP180_MB				= 0xBA ,
	BMP180_MC				= 0xBC ,
	BMP180_MD				= 0xBE
};

HAL_StatusTypeDef Init_BMP180_Calib_Param(BMP180_Calib_Param *Calib);
float BMP180_GetTempC();
int32_t	BMP180_GetPressure();
float	BMP180_GetAltitude(int32_t press);

#endif /* INC_BMP180_H_ */
