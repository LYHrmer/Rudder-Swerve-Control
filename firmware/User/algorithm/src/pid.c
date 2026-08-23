/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "pid.h"
#include "chassis_task.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define int_abs(x) ((x) > 0 ? (x) : (-x))
extern chassis_move_t chassis_move;
void ABSLimit(float *a, float abs_max){
	if(*a > abs_max)
		*a = abs_max;
	if(*a < -abs_max)
		*a = -abs_max;
}

void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

}

/**
  前馈控制
  采样周期: T = 0.001;
  转动惯量: J =1；
  摩擦系数: f = 1；
	角速度/力矩：G(s) = 1/(s + 1)
  前馈环节：Gf(s) = s + 1
	输出: out = in' + in = (in - last_in)/T + in
  */
float forwardfeed(float in)
{
	static float last_in = 0;
	float T = 0.001;
	float out;
	out = (in - last_in)/T + in;
	last_in = in ;
	return out ;
}

float PID_Calc(PidTypeDef *pid, float ref, float set)
{
  if (pid == NULL)
    {
        return 0.0f;
    }
    static float M=0.10;
	static float a=0.8;
	pid->last_out=pid->out;
	pid->Last_Dout=pid->Dout;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {   if(int_abs(set) > int_abs(pid->F_divider))
		{
					pid->Fout = pid->Kf * set;
					ABSLimit(&(pid->Fout),pid->F_out_limit);

		}
		else if(pid->F_divider!=0)
       {
	   pid->Fout =int_abs(set)/(int_abs(pid->F_divider)+1)*pid->Kf * set;
	   }


		pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
//		if(pid->mode_again==KI_SEPRATE)      //积分分离
//		{
//			if( int_abs(pid->error[0])>M)
//			{
//				pid->Iout=0.0f;
//			}
//		}
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);

		if(pid->mode_again==KD_NO_FULL)    //不完全微分
		{
		   pid->Dout=pid->Kd*(1-a)*pid->Dbuf[0]+a* pid->Last_Dout-pid->Kd*(1-a)*pid->Dbuf[1];
		}
		else
		{
		  pid->Dout = pid->Kd * pid->Dbuf[0];
		}


        pid->out = pid->Pout + pid->Iout + pid->Dout+pid->Fout;
        LimitMax(pid->out, pid->max_out);

    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
		if(pid->mode_again==KI_SEPRATE)      //积分分离
		{
			if( int_abs(pid->error[0])>M)
			{
				pid->Iout=0.0f;
			}

		}
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

		if(pid->mode_again==KD_NO_FULL||pid->flag==1)    //不完全微分
		{
		   pid->Dout=pid->Kd*(1-a)*pid->Dbuf[0]+a* pid->Last_Dout-pid->Kd*(1-a)*pid->Dbuf[1];
		}
		else
		{
		  pid->Dout = pid->Kd * pid->Dbuf[0];
		}
		 pid->out += pid->Pout + pid->Iout - pid->Dout;
        LimitMax(pid->out, pid->max_out);

}


     return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

//滑膜控制
float sat(float input,float delta)
{
	if (input > delta)
	{
		return 1;
	}
	else if (input < -delta)
	{
		return -1;
	}
	else if (int_abs(input) <=delta)
	{
		return input/delta;
	}
	   return 0;
}

void SMC_init(smc_type_def *smc,float C, float K, float eplison, float delta ,float max_out)
{
    smc->C = C;
	smc->K = K;
	smc->delta = delta;
	smc->eplison = eplison;
	smc->max_out = max_out;
	smc->fdb = 0.0f;
	smc->out = 0.0f;
	smc->set = 0.0f;
	smc->speed = 0.0f;
	smc->s = 0.0f;
}

float SMC_calc(smc_type_def *smc, float ref, float set, float speed)
{
	smc->set = set;
    smc->fdb = ref;
	smc->speed = speed;
	smc->s = smc->C*(smc->fdb-smc->set)+speed;
	smc->out = -smc->C*smc->speed - smc->eplison*sat(smc->s,smc->delta)-smc->K*smc->s;

    LimitMax(smc->out, smc->max_out);
	return smc->out;
}
