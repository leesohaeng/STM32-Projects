/*
 * HMC5883L.h
 *
 *  Created on: 2021. 12. 15.
 *      Author: sohae
 */

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

extern I2C_HandleTypeDef hi2c1;

#define HMC5883L	0x1E<<1

enum HMC5883L_regAddr
{
	// HMC5883L (Magnetometer) constants
	HMC5883L_ADDRESS        =	0x1E,

	HMC5883L_CRA			=	0x00,
	HMC5883L_CRB			=	0x01,
	HMC5883L_MR				=	0x02,
	HMC5883L_DO_X_H			=	0x03,
	HMC5883L_DO_X_L			=	0x04,
	HMC5883L_DO_Y_H			=	0x05,
	HMC5883L_DO_Y_L			=	0x06,
	HMC5883L_DO_Z_H			=	0x07,
	HMC5883L_DO_Z_L			=	0x08,
	HMC5883L_SR				=	0x09,
	HMC5883L_IR_A			=	0x0A,
	HMC5883L_IR_B			=	0x0B,
	HMC5883L_IR_C			=	0x0C
};

void	Init_HMC5883L();
uint8_t	HMC5883L_WhoAmI();
double	HMC5883L_Read();

#endif /* INC_HMC5883L_H_ */
