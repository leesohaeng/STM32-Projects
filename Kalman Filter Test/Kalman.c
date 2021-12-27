/*
 * Kalman.c
 *
 *  Created on: 2021. 12. 27.
 *      Author: sohaenglee
 */

// GYRO, ACC, COM 데이터를 Kalman필터로 처리한다.
#include "Kalman.h"

#define KDATA	9
#define PDATA   1.0
#define QDATA   0.00001  // 값이 클수록 민감.   0.00001
#define RDATA   0.00005  // 값이 작을수록 민감.  0.001
float x_next[KDATA], P_next[KDATA], z[KDATA],K[KDATA],InData[KDATA],OutData[KDATA];
float P[KDATA]; // ={PDATA,PDATA,PDATA,PDATA,PDATA,PDATA,PDATA,PDATA,PDATA};
float Q[KDATA]; // ={0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001};
float R[KDATA]; // ={0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02};
float filter_value[KDATA]; // ={0,0,0,0,0,0,0,0,0};

// kalman filter를 사용하기 전에 Indata에 데이터를 저장해야 함.
// 결과는 OutData에 저장됨.

void kalman_var_init()
{
	for(int i=0; i<KDATA; i++)
	{
		P[i] = PDATA;
		Q[i] = QDATA;
		R[i] = RDATA;
		filter_value[i] = 0;
	}
}

void kalman_filter()
{
    for(int i=0; i<KDATA; i++) {
    	x_next[i] = filter_value[i];
    	P_next[i] = P[i] + Q[i];
    	K[i] = P_next[i] / (P_next[i] + R[i]);
    	z[i] = (float)InData[i];
    	filter_value[i] = x_next[i] + K[i] * (z[i] - x_next[i]);
    	P[i] = (1-K[i])  * P_next[i];
    	OutData[i] = filter_value[i];
    }
}

