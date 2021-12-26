/*
 * ADXL345.h
 *
 *  Created on: 2021. 12. 15.
 *      Author: sohaenglee
 */

#include "stm32l4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#define	EARTH_GRAVITY_MS2			9.80665 	// m/s2
#define STANDARD_PRESSURE			1013.25 	// hPa
#define ADXL345_SCALE_MULTIPLIER    0.00390625  // 	# G/LSP

#define ADXL345				0x53<<1

enum L3G4200d_regAddr
{
	ADXL345_ADDRESS			=	0x53,

	ADXL345_DEVID			=	0x00,
	ADXL345_THRESH_TAP		=	0x1D,
	ADXL345_OFSX			=	0x1E,
	ADXL345_OFSY			=	0x1F,
	ADXL345_OFSZ			=	0x20,
	//	# Components Offset
	//	#The OFSX, OFSY, and OFSZ registers are each eight bits and
	//	#offer user-set offset adjustments in twos complement format
	//	#with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g).
	//	# Real Offset : OFS_ x 15.625
	ADXL345_DUR				=	0x21,
	ADXL345_Latent			=	0x22,
	ADXL345_Window			=	0x23,
	ADXL345_THRESH_ACT		=	0x24,
	ADXL345_THRESH_INACT	=	0x25,
	ADXL345_TIME_INACT		=	0x26,
	ADXL345_ACT_INACT_CTL	=	0x27,
	ADXL345_THRESH_FF		=	0x28,
	ADXL345_TIME_FF			=	0x29,
	ADXL345_TAP_AXES		=	0x2A,
	ADXL345_ACT_TAP_STATUS	=	0x2B,
	ADXL345_BW_RATE			=	0x2C,
	ADXL345_POWER_CTL		=	0x2D,
	ADXL345_INT_ENABLE		=	0x2E,
	ADXL345_INT_MAP			=	0x2F,
	ADXL345_INT_SOURCE		=	0x30,
	ADXL345_DATA_FORMAT		=	0x31,
	//	# Register 0x31 - Data Format - Read/Write
	//	# D7: SELF_TEST	D6: SPI	D5: INT_INVERT	D4: 0	D3: FULL_RES	D2:Justify	D1-D0: Range
	//	# D1-D0 = 00: +/-2G	D1-D0 = 01: +/-4G	D1-D0 = 10: +/-8G	D1-D0 = 11: +/-16G
	//	# Range:
	//	#	FULL_RES=1: 3.9 mG/LSP (0.00390625 G/LSP)
	//	#	+/-2G,10bit mode:  3.9 mG/LSP
	//	#	+/-4G,10bit mode:  7.8 mG/LSP
	//	#	+/-8G,10bit mode:  15.6 mG/LSP
	//	#	+/-16G,10bit mode:  31.2 mG/LSP
	ADXL345_DATAX0			=	0x32,
	ADXL345_DATAX1			=	0x33,
	ADXL345_DATAY0			=	0x34,
	ADXL345_DATAY1			=	0x35,
	ADXL345_DATAZ0			=	0x36,
	ADXL345_DATAZ1			=	0x37,
	ADXL345_FIFO_CTL		=	0x38,
	ADXL345_FIFO_STATUS		=	0x39,

	ADXL345_BW_RATE_3200HZ	= 0x0F,
	ADXL345_BW_RATE_1600HZ	= 0x0E,
	ADXL345_BW_RATE_800HZ	= 0x0D,
	ADXL345_BW_RATE_400HZ	= 0x0C,
	ADXL345_BW_RATE_200HZ	= 0x0B,
	ADXL345_BW_RATE_100HZ	= 0x0A,              // # (default)
	ADXL345_BW_RATE_50HZ	= 0x09,
	ADXL345_BW_RATE_25HZ	= 0x08,

	ADXL345_RANGE_2G		= 0x00,              // # +/-  2g (default)
	ADXL345_RANGE_4G		= 0x01,              // # +/-  4g
	ADXL345_RANGE_8G		= 0x02,              // # +/-  8g
	ADXL345_RANGE_16G		= 0x03,              // # +/- 16g

	ADXL345_MEASURE			= 0x08
};


void	Init_ADXL345();
uint8_t	ADXL345_WhoAmI();
void	ADXL345_Read(ACCDATA *G);

#endif /* INC_ADXL345_H_ */
