#include "chassis_task.h"
#include "power_control.h"
#include "referee.h"
#include "user_lib.h"
#include "detect_task.h"
#include "math.h"

/****************************************
*函 数 名: power_forecast
*功能说明: 功率模型计算
*形    参: K_0 系数k0
					 K_1 系数k1
					 K_2 系数k2
					 A 静态损耗
					 Current 转子转矩电流
					 Omega 转子角速度
*返 回 值: 模型计算所得功率
*****************************************/
float power_forecast(float K_0, float K_1, float K_2, float A, float Current, float Omega)
{
	return (K_0 * Current * Omega + K_1 * Omega * Omega + K_2 * Current * Current + A);
}
/****************************************
*函 数 名: Power_reso_M3508
*功能说明: 根据功率反解电流控制值
*形    参: K_0 系数k0
					 K_1 系数k1
					 K_2 系数k2
					 A 静态损耗
					 Current 转子转矩电流
					 Omega 转子角速度
					 GivePower 给定功率
*返 回 值: 无
*****************************************/

void Power_reso_M3508(float K_0, float K_1, float K_2, float A,float Current,float Omega,float GivePower)
{
	float a,b,c,delta,h;
	float real1,real2;
	a = K_2;
	b = K_0 * Omega;
	c = K_1 * Omega * Omega + A - GivePower;
	delta = b * b - 4 * a * c;
	h = sqrt(b * b - 4 * a * c);
	if(Current > 0)
	{
		 Current = (-b + h)/(2 * a);
		 if (Current > MAX_MOTOR_CAN_CURRENT) Current = MAX_MOTOR_CAN_CURRENT;

	}
	else
	{
		 Current = (-b - h)/(2 * a);
		 if (Current < -MAX_MOTOR_CAN_CURRENT) Current = -MAX_MOTOR_CAN_CURRENT;
	}

}
/****************************************
*函 数 名: Power_reso_GM6020
*功能说明: 根据功率反解电流控制值
*形    参: K_0 系数k0
					 K_1 系数k1
					 K_2 系数k2
					 A 静态损耗
					 Current 转子转矩电流
					 Omega 转子角速度
					 GivePower 给定功率
*返 回 值: 无
*****************************************/
int16_t Power_reso_GM6020(float K_0, float K_1, float K_2, float A, float Omega,float GivePower)
{
	float a,b,c,delta,h;
	float real1,real2;
	int16_t Current;
	a = K_2;
	b = K_0 * Omega;
	c = K_1 * Omega * Omega + A - GivePower;
	delta = b * b - 4 * a * c;
	h = sqrt(b * b - 4 * a * c);
	if(delta < 0)
	{
		Current = 0;
	}
	else
	{
		real1 = (-b + h) / (2 * a);
		real2 = (-b - h) / (2 * a);
		if((real1 > 0.0f&&real2 < 0.0f) || (real1 < 0.0f&&real2 > 0.0f))
		{
			if((Current > 0.0f&&real1 > 0.0f)||(Current < 0.0f&&real1 < 0.0f))
			{
				Current = real1 * GM6020_Current_To_Out;
			}
			else
			{
				Current = real2 * GM6020_Current_To_Out;
			}
		}
		else
		{
			if(fabs(real1) < fabs(real2))
			{
				Current = real1 * GM6020_Current_To_Out;
			}
			else
			{
				Current = real2 * GM6020_Current_To_Out;
			}
		}
	}
	return Current;
}
