#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"


typedef struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
		float slide[8] ;    //滑动滤波数组
} ramp_function_source_t;

//斜波函数升级版
typedef struct
{
		float target;        //目标值
		float current_velocity;   //当前速度
    float Decrease_Value;    //每周期减小的最小值
    float Increase_Value;    //每周期增大的最大值
		float out;          //输出数据
} ramp_function_source_t_2;


typedef struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;

//快速开方
extern float invSqrt(float num);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
void ramp_init_2(ramp_function_source_t_2 *ramp_source_type, float Increase_Value ,float Decrease_Value );
void ramp_calc_2(ramp_function_source_t_2 *ramp_source_type,float target);
//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);
//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
//绝对限制
extern void abs_limit(float *num, float Limit);
//判断符号位
extern float sign(float value);
//浮点死区
extern float fp32_deadline(float Value, float minValue, float maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern float fp32_constrain(float Value, float minValue, float maxValue);
//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
extern float loop_fp32_constrain(float Input, float minValue, float maxValue);
//角度 °限幅 180 ~ -180
extern float theta_format(float Ang);
extern float Ramp_float(float target_value,float current_value,float step_length);
//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
