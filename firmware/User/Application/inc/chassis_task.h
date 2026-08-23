#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "slope.h"
#include "pid.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "stm32.h"
#include "main.h"
#include "stdbool.h"
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

#define DEG2R(x) ((x)*PI /180.0f)
// rpm换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2

//底盘低通滤波系数
#define CHASSIS_ACCEL_X_NUM 0.02f
#define CHASSIS_ACCEL_Y_NUM 0.01f

//X，Y角度与遥控器输入比例
#define X_RC_SEN   0.0006f
#define Y_RC_SEN -0.0005f //0.005

//Y,Y角度和鼠标输入的比例
#define X_Mouse_Sen 0.0002f
#define Y_Mouse_Sen 0.00025f

//云台编码值零点
#define GIMBAL_ENCODE_ZERO 4614

//编码-弧度
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.00076708403f //      2*  PI  /8191
#endif

//底盘电机最大速度
#define MAX_WHEEL_SPEED 50.0f
// #define MAX_WHEEL_SPEED 40.0f
// #define MAX_WHEEL_SPEED 20.0f

//
// 功率控制相关参数
// 3508电机
#define toque_coefficient 1.99688994e-6f 	// (20/16384)*(0.3)*(187/3591)/9.55  此参数将电机电流转换为扭矩
#define k2 1.23e-07						 	// 铜损系数
#define k1 1.453e-07					 	// 磁损系数
#define constant_3508 	4.081f  		 	// 控制器静态误差(要改)
//6020电机
#define GM6020_K0 0.8130f
#define GM6020_K1 -0.0005f
#define GM6020_K2 6.0021f
#define GM6020_K3 1.3715f
#define GM6020_Current_To_Out (3.0f/16384.0f)
//转换式
#define RpmToOmega(rpm) (rpm*(float)PI/30.0f)
#define OmegaToRpm(omega) (omega *30.0f /(float)PI)
#define Torque_To_Icmd(torque) (torque / KT * M3508_Current_To_Out)
#define Icmd_To_Torque(icmd) (icmd * KT /M3508_Current_To_Out)
#define Encoder_To_PI(ecd) ((ecd/4095.5f) * PI - PI)

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16384.0f

//各舵电机零点
#define Forward_L_ecd 0  	// 左前轮  1号
#define Forward_R_ecd 7491	// 右前轮  2号(半舵使用)
#define Back_L_ecd 2700			// 左后轮  3号(半舵使用)//3399
#define Back_R_ecd 0	   		// 右后轮  4号

//底盘电机速度环PID
#define CHASSIS_KP 2000.f  //2000
#define CHASSIS_KI 0.0f //0
#define CHASSIS_KD 20.f //20
#define CHASSIS_MAX_OUT 16384.f
#define CHASSIS_MAX_IOUT 500.f

//舵电机PID
#define RUDDER_P_P 10.0f     //19.0
#define RUDDER_P_I 0.00f      //0
#define RUDDER_P_D 0.0f      //2.0
#define RUDDER_P_N 0.01f      //0.01.0
#define RUDDER_S_P 1.5f     //3.0 1.5//8
#define RUDDER_S_I 0.0f      //0.0
#define RUDDER_S_D 0.0f      //0.0
#define RUDDER_S_N 0.0f      //0.0

//底盘跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP  10.0f//3.0f               // 3.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI  0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD  200.0f//20.f//10.0f 90
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT   15//4
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KF 0.0f
#define CHASSIS_FOLLOW_GIMBAL_F_divider 0.0
#define CHASSIS_FOLLOW_GIMBAL_F_out_limit 0.0f

//底盘电机功率环PID
#define M3505_MOTOR_POWER_PID_KP 1.4f//1.0f
#define M3505_MOTOR_POWER_PID_KI 0.0f
#define M3505_MOTOR_POWER_PID_KD 0.1f
#define M3505_MOTOR_POWER_PID_MAX_OUT 10.0f  //60
#define M3505_MOTOR_POWER_PID_MAX_IOUT 0.0f

//开启超电时的放电大小参数
//#define CAP_OUTPUT_to_CHASSIS 8
//#define CAP_OUTPUT_to_CHASSIS_FLY 7

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
//1/60*2PI/减速比(目前为268/17)*轮半径
#define M3508_MOTOR_RPM_TO_VECTOR 0.00415166939606860113913751068997
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

typedef enum {
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   		//底盘跟随云台
	CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  		//底盘自主
	CHASSIS_VECTOR_SPIN,                		//小陀螺
	CHASSIS_VECTOR_NO_FOLLOW_YAW,       		//底盘不跟随
	CHASSIS_VECTOR_RAW,							//底盘原始控制
	RUDDER_VECTOR_FOLLOW_GIMBAL_YAW     		//舵跟随云台
} chassis_mode_e;

typedef struct {
	float relative_angle;
	int16_t relative_angle_receive;
	float pitch_abs;
} Gimbal_data;

typedef struct {
	float totalCurrentTemp;
	float totalSpeed;
	float current[4];
	float power_current[4];
	float speed[4];
	float POWER_MAX;
	float MAX_current[4];
	float SPEED_MIN;
	float K;
	float power_charge; //超电充电
	float forecast_motor_power[4]; // 预测单个电机功率
	float forecast_total_power; // 预测总功率
	uint8_t cap_flag;
	struct {
		float power_scale[4];
		float sumPowerCmd;//预测舵电机功率
		float alloctablePower;//最终可利用的功率
		float alloctableSumPower;//最终可用的总功率
	}rudder_;
} Power_Control;


typedef struct {
	const motor_measure_t* rudder_motor_measure;   //云台数据
	Rudder_control rudder_control;
	uint16_t offset_ecd;
	float max_relative_angle; //rad
	float min_relative_angle; //rad

	float control;
	float motor_gyro;         //rad/s
	float motor_gyro_set;    //角速度设定
	float motor_speed;
	float raw_cmd_current;
	float current_set;
	int16_t given_current;
	int16_t  ecd_add;
	int16_t last_ecd_add;
	int16_t ecd_temp_error;
	int16_t ecd_set;
	int16_t ecd_set_final;
	int16_t last_ecd_set;
	int16_t ecd_error;
	int16_t ecd_error_true;
	int16_t ecd_turn;
	int16_t ecd_change_MIN;
	float wheel_speed;
	float rudder_angle;
	int16_t ecd_zero_set;
	int8_t Judge_Speed_Direction;
	float Judge_Speed_cosk;
} Rudder_Motor_t;

typedef struct {
	const motor_measure_t* chassis_motor_measure;
	float accel;
	float speed;
	float speed_set;0f
	int16_t give_current;
	PidTypeDef chassis_pid;
} Chassis_Motor_t;

typedef struct {
	Slope_t Slope_X_Speed;   //斜坡函数
	Slope_t Slope_Y_Speed;
	ramp_function_source_t  Slope_CAP_Speed;
	motor_measure_t yaw;
	Chassis_Motor_t motor_chassis[4];
	chassis_mode_e chassis_motor_mode;
	chassis_mode_e last_chassis_motor_mode;

	PidTypeDef chassis_angle_pid;   //底盘跟随角度pid
	PidTypeDef buffer_pid;     //功率环PID
	Power_Control  power_control;
	Power_Control  rudder_power_control;

	float POWER_MAX_SET;
	Gimbal_data gimbal_data;
	Rudder_Motor_t Forward_L;
	Rudder_Motor_t Forward_R;
	Rudder_Motor_t Back_R;
	Rudder_Motor_t Back_L;

	/**
	 *			^x
	 *			|
	 *			|
	 * 			|
	 * 	y<------`俯视
	 *
	 */


	float vx;    //底盘速度 前进方向 前为正，单位 m/s
	float vy;   //底盘速度 左右方向 左为正  单位 m/s
	float wz;  //底盘旋转角速度，逆时针为正 单位 rad/s

	float vx_set;
	float vy_set;
	float wz_set;
	float chassis_relative_angle_set; //设置相对云台控制角度

	float chassis_relative_last;

	int16_t vx_temp;
	int16_t vy_temp;

	float vx_set_CANsend;
	float vy_set_CANsend;
	float wz_set_CANsend;
	int16_t chassis_mode_CANsend;		 //模式
	float chassis_power;

	int16_t chassis_power_MAX;    //裁判系统最大功率，双板通信数据
	int16_t chassis_power_buffer; //缓冲能量，双板通信数据

	int16_t key_C;

	first_order_filter_type_t chassis_cmd_slow_set_vx;   // 滤波数据
	first_order_filter_type_t chassis_cmd_slow_set_vy;

	ramp_function_source_t vx_ramp;
	ramp_function_source_t vy_ramp;

	float rudder_given_current[4];
	float rudder_omega[4];
	float rudder_torque_current[4];
	float Encoder_add[4];

	uint8_t key_shift;
	uint8_t ready_flag;
}chassis_move_t;

extern chassis_move_t chassis_move;
extern void chassis_task(void const* pvParameters);
void UI_task(void const* argument);
void chassis_rc_to_control_vector(float* vx_set, float* vy_set, chassis_move_t* chassis_move_rc_to_vector);

#endif
