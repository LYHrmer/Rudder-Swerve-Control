#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "pid.h"
#include "stm32.h"
#include "math.h"
#include "arm_math.h"

#define M3508_K0 2.49688994e-6f
#define M3508_K1 1.253e-07
#define M3508_K2 1.23e-07
#define M3508_K3 4.081f

#define GM6020_K0 0.8130f
#define GM6020_K1 -0.0005f
#define GM6020_K2 6.0021f
#define GM6020_K3 1.3715f

#define Encoder 8191
#define Encoder_Half 8191/2
//3508新减速比
#define Reduction_ratio 18
#define KT 0.01526f//N*m/A 0.3*187/3591力矩电流常数
// 电流到输出的转化系数
#define M3508_Current_To_Out (20.0f/16384.0f)
#define GM6020_Current_To_Out (3.0f/16384.0f)
#define M2006_Current_To_Out (10.0f/10000.0f)
//转换式
#define RpmToOmega(rpm) (rpm*(float)PI/30.0f)
#define OmegaToRpm(omega) (omega *30.0f /(float)PI)
#define Torque_To_Icmd(torque) (torque / KT * M3508_Current_To_Out)
#define Icmd_To_Torque(icmd) (icmd * KT /M3508_Current_To_Out)
#define Encoder_To_PI(ecd) ((ecd/4095.5f) * PI - PI)

#define M3508_RPM_TO_VECTOR  0.00415166939606860113913751068997
#define GM6020_RPM_TO_VECTOR 0.001746201886833
#define M2006_RMP_TO_VECTOR 0.00290888208665721596153948461415f


/*****3508解析数据*****/
typedef struct
{
	/*****大疆电机源数据*****/
	 struct
	{
			uint16_t ecd;
			int16_t speed_rpm;
			int16_t given_current;
			uint8_t temperate;
			int16_t last_ecd;
	} motor_measure_t;

	PidTypeDef chassis_pid;
	/*******系数*******/
	float k0;
	float k1;
	float k2;
	float k3;
	/*******编码值*******/
	int encoder_round;
	float get_encoder;
	/*******给定值*******/
	float give_speed_set;
	float give_speed;
	float give_angle_set;
	float give_angle;
	int16_t give_cmd_current;
	float give_torque;
	float give_power;
	/*******当前值*******/
  float get_speed;
	float get_omega;
	float get_angle;
	float get_torque_current;//转矩电流
	float get_torque;//转子扭矩
	float get_power;
	float get_wheel_angle;

} M3508_Motor_t;

/*****6020解析数据*****/
typedef struct
{
		/*****大疆电机源数据*****/
	struct
	{
			uint16_t ecd;
			int16_t speed_rpm;
			int16_t given_current;
			uint8_t temperate;
			int16_t last_ecd;
	} motor_measure_t;
	Rudder_control rudder_control;
	/*******系数*******/
	float k0;
	float k1;
	float k2;
	float k3;
	/*******编码值*******/
	int16_t encoder_round;
	float get_encoder;
	float Encoder_Offset;   //舵电机归中值
	float Encoder_set;
	float Encoder_add;
	float last_Encoder_add;
	float Encoder_error;
	float Speed_Dir;
	float Speed_cosk;
	/*******给定值*******/
	float give_speed_set;
	float give_speed;
	float give_angle;
	float last_give_angle;
	float give_cmd_current;
	float give_torque;
	float give_power;
	/*******当前值*******/
	float get_speed;
	float get_omega;
	float get_angle;
	int16_t get_torque_current;
	float get_torque;
	float get_power;

} GM6020_Motor_t;

typedef struct
{
		/*
				 /|\
		0	  	|	   1
					|
	 <------|-------
					|
		2     |    3
	*/
		/*****大疆电机源数据*****/
 struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;
	/*******系数*******/
	float k0;
	float k1;
	float k2;
	float k3;
	/*******编码值*******/
	int16_t encoder_round;
	float get_encoder;
	float Encoder_set;

	/*******给定值*******/
	float give_speed_set;
	float give_speed;
	float give_angle_set;
	float give_angle;
	float last_give_angle;
	int16_t give_cmd_current;
	float give_torque;
	float give_power;
	/*******当前值*******/
	float get_speed;
	float get_omega;
	float get_angle;
	float get_torque_current;
	float get_torque;
	float get_power;

} M2006_Motor_t;



extern void M3508_Init(M3508_Motor_t *motor);
extern void M3508_Rx_Date(M3508_Motor_t *motor);
extern void GM6020_Init(GM6020_Motor_t *motor,float ecd_offect);
extern void GM6020_Rx_Date(GM6020_Motor_t *motor);
extern void M2006_Init(M2006_Motor_t *motor);
extern void M2006_Rx_Date(M2006_Motor_t *motor);
#endif
