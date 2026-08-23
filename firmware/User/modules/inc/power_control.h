#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H
#include "main.h"
#include "stdbool.h"

//typedef struct
//{
//	struct
//	{
//		float power_scale[4];
//		float sumError;//error的和
//		float sumPowerCmd;//预测舵电机功率
//		float alloctablePower;//最终可利用的功率
//	}rudder_;
//	struct
//	{
//		float power_scale[4];
//		float sumError;//error的和
//		float sumPowerCmd;//预测轮电机功率
//		float alloctablePower;//最终可利用的功率
//	}wheel_;
//} Power_Control;


extern float power_forecast(float K_0, float K_1, float K_2, float A, float Current, float Omega);

extern void Power_reso_M3508(float K_0, float K_1, float K_2, float A,float Current,float Omega,float GivePower);
extern int16_t Power_reso_GM6020(float K_0, float K_1, float K_2, float A, float Omega,float GivePower);

#endif
