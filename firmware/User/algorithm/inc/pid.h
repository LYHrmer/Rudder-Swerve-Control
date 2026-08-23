#ifndef PID_H
#define PID_H
#include "main.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};
enum PID_MODE_AGAIN
{
    KI_SEPRATE = 0, //积分分离
    KD_NO_FULL,     //不完全微分
};
typedef struct
{
    uint8_t mode;
	uint8_t mode_again;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;
	float Kf;

    float max_out;  //最大输出
    float max_iout; //最大积分输出


    float set;
    float fdb;

    float out;
	float last_out;
    float Pout;
    float Iout;
    float Dout;
	float Last_Dout;
	float Fout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
    int flag;
	float F_divider;//前馈分离
	float F_out_limit;//前馈限幅

} PidTypeDef;
typedef struct
{
    //smc 三参数
    float C;         //c越大，收敛速度越快
    float delta;     //delta(0，1]为饱和函数的边界层，用于抑制抖震
    float eplison;  //用于控制稳态误差，epsilon越大稳态误差越小

    float max_out;  //最大输出
    float set;
    float fdb;
	float speed;
    float out;
	float s;       //滑模面
	float K;      //为控制状态不离开滑模面的增益
} smc_type_def;
extern void PID_Init(PidTypeDef* pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_Calc(PidTypeDef *pid, float ref, float set);
extern void PID_clear(PidTypeDef *pid);
float forwardfeed(float in);
extern void SMC_init(smc_type_def *smc,float C,float K, float eplison, float delta ,float max_out);
extern float SMC_calc(smc_type_def *smc, float ref, float set, float speed);
#endif
