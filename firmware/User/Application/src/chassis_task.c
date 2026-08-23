#include "chassis_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "arm_math.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "chassis_behaviour.h"
#include "stm32.h"
#include "math.h"
#include "detect_task.h"
#include "referee.h"
#include "slope.h"
#include "power_control.h"
#include "UI_task.h"
#include "cmsis_os.h"

extern cap_measure_t get_cap;
// 轮组半径
const float Wheel_Radius = 0.0625f;
// 轮距中心长度
const float Wheel_To_Core_Distance[4] = { 0.239f,
										 0.239f,
										 0.239f,
										 0.239f, };
// 轮组方位角
const float Wheel_Azimuth[4] = { 7.0f * PI / 4.0f,
								5.0f * PI / 4.0f,
								3.0f * PI / 4.0f,
								1.0f * PI / 4.0f, };

static const float motor_speed_pid[3] = {
	 CHASSIS_KP, CHASSIS_KI, CHASSIS_KD };
static const float power_buffer_pid[3] = {
	M3505_MOTOR_POWER_PID_KP, M3505_MOTOR_POWER_PID_KI, M3505_MOTOR_POWER_PID_KD }; //功率环PID参数
static const float chassis_yaw_pid[3] = {
	CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD };
static const float rudder_pid[8] = {
	RUDDER_P_P, RUDDER_P_I, RUDDER_P_D, RUDDER_P_N, RUDDER_S_P, RUDDER_S_I, RUDDER_S_D, RUDDER_S_N };

static uint8_t ui_init = 0;

#ifndef PI
#define PI 3.14159265358979f
#endif
#define HALF_PI (PI*0.5)


/**
 * @brief 	        角度转弧度
 * @param[in]      angle：输入角度
 */
#define rad(angle) angle = angle / 180 * 3.1415926f

 // 底盘控制所有相关数据
chassis_move_t chassis_move;
int16_t key_ctrl;
float kx = 1.f, ky = 1.f, kw = 1.f; // 速度转换的几个系数

static uint8_t referee_last_state;
static uint8_t referee_ready;
static uint16_t cnt;
static uint8_t cnt_flag;
/**
 * @brief          返回舵电机 6020电机数据指针
 * @author         XQL
 * @param[in]      none
 * @retval         电机数据指针
 */
const Rudder_Motor_t* get_Forward_L_motor_point(void) {
	return &chassis_move.Forward_L;
}
const Rudder_Motor_t* get_Forward_R_motor_point(void) {
	return &chassis_move.Forward_R;
}
const Rudder_Motor_t* get_Back_R_motor_point(void) {
	return &chassis_move.Back_R;
}
const Rudder_Motor_t* get_Back_L_motor_point(void) {
	return &chassis_move.Back_L;
}

// 舵电机PID初始化
static void RUDDER_PID_INIT(chassis_move_t* rudder_init, const float PID[8]);
// 轮电机速度设置
static void chassis_speed_control_set(chassis_move_t* chassis_speed_set);
// 底盘初始化
static void chassis_init(chassis_move_t* chassis_move_init);
// 底盘数据更新
static void chassis_feedback_update(chassis_move_t* chassis_move_update);
// 底盘模式设置
static void chassis_set_mode(chassis_move_t* chassis_move_mode);
// 底盘切换模式状态保存
static void chassis_mode_change_control_transit(chassis_move_t* chassis_move_transit);
// 底盘控制量设置
static void chassis_set_contorl(chassis_move_t* chassis_move_control);

static void chassic_rudder_preliminary_A_S_solution(chassis_move_t* chassic_rudder_preliminary_solution);
// 舵控制输入
static void rudder_control_loop(chassis_move_t* rudder_move_control_loop);
// 轮电机控制输出
static void CHASSIC_MOTOR_PID_CONTROL(chassis_move_t* chassis_motor);
// 舵电机控制输出
static void RUDDER_MOTOR_PID_CONTROL(Rudder_Motor_t* rudder_motor);
// 舵电机输出角
static void Rudder_motor_relative_angle_control(Rudder_Motor_t* chassis_motor);

// 舵电机功率控制
static void RUDDER_POWER_CONTROL(chassis_move_t* rudder_power);
// 轮电机动态功率控制
void chassis_power_move_control(chassis_move_t* chassis_motor);
// 超级电容充电
void Power_Charge(float power);
void cap_mode_loop(Power_Control* cap_mode_control);
// 舵轮运动学正解
static void chassis_Self_Resolution(chassis_move_t* chassis_Self_Res);


/**
 * @brief          底盘任务
 * @author         LYH
 * @param[in]      chassis_move_init：底盘数据指针
 * @retval         none
 */
void chassis_task(void const* pvParameters) {
	//初始化底盘控制结构体
	chassis_init(&chassis_move);
	RefereeInit(&referee, &huart6);
	while (1) {
		if (referee.game_robot_state_.power_management_chassis_output == 1 &&
			referee_last_state == 0 && cnt_flag == 0) {
			cnt_flag = 1;         //计数标志位置1
			referee_ready = 0;    //裁判系统标志位置0
		}
		//开始计数
		if (cnt_flag) {
			cnt++;
			if (cnt > 1) {
				cnt_flag = 0;       //复活
				referee_ready = 1;
				cnt = 0;
			}
		}
		//根据底盘控制结构体中的模式进入对应模式状态机
		chassis_set_mode(&chassis_move);
		//对应模式的再次初始化
		chassis_mode_change_control_transit(&chassis_move);
		//读取反馈
		chassis_feedback_update(&chassis_move);
		//控制量处理
		chassis_set_contorl(&chassis_move);
		//舵轮控制循环
		rudder_control_loop(&chassis_move);
		//舵轮功率控制循环
		RUDDER_POWER_CONTROL(&chassis_move);
		//底盘电机控制
		CHASSIC_MOTOR_PID_CONTROL(&chassis_move);

		// if (chassis_move.chassis_mode_CANsend == GIMBAL_DISCONNECT ||
		//     chassis_move.chassis_mode_CANsend == NO_MOVE ||
		//     referee.game_robot_state_.power_management_chassis_output ==
		//         0 ||
		//     !referee_ready) // 若双板通信没接收到数据，则舵和轮都不动
		if (chassis_move.chassis_mode_CANsend == GIMBAL_DISCONNECT
			|| chassis_move.chassis_mode_CANsend == NO_MOVE) //
			// 若双板通信没接收到数据，则舵和轮都不动
		{
			CAN_cmd_chassis(0, 0, 0, 0);
			CAN_cmd_rudder(0, 0, 0, 0);
			CAN_board_send(referee.game_robot_state_.shooter_barrel_heat_limit, referee.game_robot_state_.shooter_barrel_cooling_value,
							   referee.power_heat_data_.shooter_17mm_barrel_heat, get_gyro_data_point()[2], referee.game_robot_state_.robot_id, referee.game_robot_state_.power_management_shooter_output);
			CAN_CMD_cap(0, 1);
		}
		else {
			CAN_board_send(referee.game_robot_state_.shooter_barrel_heat_limit, referee.game_robot_state_.shooter_barrel_cooling_value,
							   referee.power_heat_data_.shooter_17mm_barrel_heat, get_gyro_data_point()[2], referee.game_robot_state_.robot_id, referee.game_robot_state_.power_management_shooter_output);
			CAN_cmd_rudder(chassis_move.rudder_given_current[0], chassis_move.rudder_given_current[1], chassis_move.rudder_given_current[2], chassis_move.rudder_given_current[3]);
			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current, chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
			CAN_CMD_cap(chassis_move.power_control.power_charge, 0);
		}
		referee_last_state =
			referee.game_robot_state_.power_management_chassis_output;
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);

		if (rece_.reset_flag) {
			HAL_NVIC_SystemReset();
		}
	}
}


/**
 * @brief          初始化底盘数据
 * @author         XQL 1.0
 *                 LYH 2.0
 * @param[in]      chassis_move_init：底盘数据指针
 * @retval         none
 */
static void chassis_init(chassis_move_t* chassis_move_init) {
	if (chassis_move_init == NULL) {
		return;
	}
	// 舵电机数据指针获取
	chassis_move_init->Forward_L.rudder_motor_measure = get_Forward_L_motor_measure_point();
	chassis_move_init->Forward_R.rudder_motor_measure = get_Forward_R_motor_measure_point();
	chassis_move_init->Back_R.rudder_motor_measure = get_Back_R_motor_measure_point();
	chassis_move_init->Back_L.rudder_motor_measure = get_Back_L_motor_measure_point();

	// 舵电机PID初始化
	RUDDER_PID_INIT(chassis_move_init, rudder_pid);

	// 舵电机编码值初始化
	chassis_move.Forward_L.ecd_zero_set = Forward_L_ecd;
	chassis_move.Forward_R.ecd_zero_set = Forward_R_ecd;
	chassis_move.Back_R.ecd_zero_set = Back_R_ecd;
	chassis_move.Back_L.ecd_zero_set = Back_L_ecd;

	chassis_move_init->chassis_motor_mode = CHASSIS_VECTOR_RAW;

	// 轮电机数据指针获取，PID初始化
	int i;
	for (i = 0; i < 4; i++) {
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_Init(&chassis_move_init->motor_chassis[i].chassis_pid, PID_POSITION, motor_speed_pid, CHASSIS_MAX_OUT, CHASSIS_MAX_IOUT);
	}

	//底盘跟随云台PID
	PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	//功率环PID
	PID_Init(&chassis_move_init->buffer_pid, PID_POSITION, power_buffer_pid, M3505_MOTOR_POWER_PID_MAX_OUT, M3505_MOTOR_POWER_PID_MAX_IOUT);
	//底盘斜坡函数初始化
	slope_init(&chassis_move_init->Slope_X_Speed, 25.0f / 1000.0f, 20.0f / 200.0f, Slope_First_REAL);
	slope_init(&chassis_move_init->Slope_Y_Speed, 25.0f / 1000.0f, 20.0f / 200.0f, Slope_First_REAL);
	ramp_init(&chassis_move_init->Slope_CAP_Speed, 0.1, 400, 0);
	// 轮电机转动方向初始化
	chassis_move_init->Forward_L.Judge_Speed_Direction = chassis_move_init->Forward_R.Judge_Speed_Direction =
		chassis_move_init->Back_L.Judge_Speed_Direction = chassis_move_init->Back_R.Judge_Speed_Direction = 1;
	// 底盘数据初始化
	chassis_move_init->chassis_relative_last = 0.0f;
	chassis_feedback_update(chassis_move_init);
	// 斜坡函数初始化
//   ramp_init(&chassis_move_init->vx_ramp, 0.030f, 20, -20);
//   ramp_init(&chassis_move_init->vy_ramp, 0.030f, 10, -10);
	chassis_move.power_control.SPEED_MIN = 0.1f;
}

/**
 * @brief          更新底盘数据
 * @author         XQL  1.0
 *                 LYH  2.0
 * @param[in]      chassis_move_update：底盘数据指针
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t* chassis_move_update) {
	if (chassis_move_update == NULL) {
		return;
	}
	uint8_t i = 0;
	for (i = 0; i < 4; i++) {
		//轮的实际转速
		chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
	}

	chassis_move_update->Forward_L.motor_speed = chassis_move_update->Forward_L.rudder_motor_measure->speed_rpm;
	chassis_move_update->Forward_R.motor_speed = chassis_move_update->Forward_R.rudder_motor_measure->speed_rpm;
	chassis_move_update->Back_L.motor_speed = chassis_move_update->Back_L.rudder_motor_measure->speed_rpm;
	chassis_move_update->Back_R.motor_speed = chassis_move_update->Back_R.rudder_motor_measure->speed_rpm;

	//角速度
	chassis_move_update->rudder_omega[0] = RpmToOmega(chassis_move_update->Forward_L.rudder_motor_measure->speed_rpm);
	chassis_move_update->rudder_omega[2] = RpmToOmega(chassis_move_update->Back_L.rudder_motor_measure->speed_rpm);
	chassis_move_update->rudder_omega[3] = RpmToOmega(chassis_move_update->Back_R.rudder_motor_measure->speed_rpm);
	chassis_move_update->rudder_omega[1] = RpmToOmega(chassis_move_update->Forward_R.rudder_motor_measure->speed_rpm);
	//编码值变量
	chassis_move_update->Encoder_add[0] = chassis_move_update->Forward_L.ecd_add;
	chassis_move_update->Encoder_add[2] = chassis_move_update->Back_L.ecd_add;
	chassis_move_update->Encoder_add[3] = chassis_move_update->Back_R.ecd_add;
	chassis_move_update->Encoder_add[1] = chassis_move_update->Forward_R.ecd_add;

	// 云台的相对角度
	chassis_move.gimbal_data.relative_angle_receive = chassis_move_update->yaw.ecd - GIMBAL_ENCODE_ZERO;

	//中心化，过圈处理
	if (chassis_move_update->gimbal_data.relative_angle_receive > 4096) {
		chassis_move_update->gimbal_data.relative_angle_receive -= 8192;
	}
	else if (chassis_move_update->gimbal_data.relative_angle_receive < -4096) {
		chassis_move_update->gimbal_data.relative_angle_receive += 8192;
	}

	//底盘相对云台角度[-PI, PI]
	chassis_move_update->gimbal_data.relative_angle = ((float)(chassis_move_update->gimbal_data.relative_angle_receive)) * Motor_Ecd_to_Rad;

	//w_z计算
	chassis_Self_Resolution(chassis_move_update);
}

/**
 * @brief          底盘控制模式设置
 * @author         XQL
 * @param[in]      chassis_move_mode：底盘数据指针
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t* chassis_move_mode) {
	if (chassis_move_mode == NULL) {
		return;
	}
	chassis_behaviour_mode_set(chassis_move_mode);
}

/**
 * @brief          底盘切换模式数据缓存
 * @author         XQL 1.0
 *                 LYH 2.0
 * @param[in]      chassis_move_transit：底盘数据指针
 * @retval         none
 */
static void chassis_mode_change_control_transit(chassis_move_t* chassis_move_transit) {
	if (chassis_move_transit == NULL || chassis_move_transit->last_chassis_motor_mode == chassis_move_transit->chassis_motor_mode) {
		return;
	}

	if ((chassis_move_transit->last_chassis_motor_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && // 切换到底盘跟随云台
		chassis_move_transit->chassis_motor_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) {
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		// 更新舵电机零点
		chassis_move_transit->Forward_L.ecd_zero_set = Forward_L_ecd;
		chassis_move_transit->Forward_R.ecd_zero_set = Forward_R_ecd;
		chassis_move_transit->Back_R.ecd_zero_set = Back_R_ecd;
		chassis_move_transit->Back_L.ecd_zero_set = Back_L_ecd;
		// 更正轮电机转向
		chassis_move_transit->Forward_L.Judge_Speed_Direction = chassis_move_transit->Forward_R.Judge_Speed_Direction =
			chassis_move_transit->Back_L.Judge_Speed_Direction = chassis_move_transit->Back_R.Judge_Speed_Direction = 1;
	}
	else if ((chassis_move_transit->last_chassis_motor_mode != CHASSIS_VECTOR_SPIN) && // 切换到小陀螺模式
			 chassis_move_transit->chassis_motor_mode == CHASSIS_VECTOR_SPIN) {
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		// 更新舵电机零点
		chassis_move_transit->Forward_L.ecd_zero_set = Forward_L_ecd;
		chassis_move_transit->Forward_R.ecd_zero_set = Forward_R_ecd;
		chassis_move_transit->Back_R.ecd_zero_set = Back_R_ecd;
		chassis_move_transit->Back_L.ecd_zero_set = Back_L_ecd;
		// 更正轮电机转向
		chassis_move_transit->Forward_L.Judge_Speed_Direction = chassis_move_transit->Forward_R.Judge_Speed_Direction =
			chassis_move_transit->Back_L.Judge_Speed_Direction = chassis_move_transit->Back_R.Judge_Speed_Direction = 1;
	}
	else if ((chassis_move_transit->last_chassis_motor_mode != RUDDER_VECTOR_FOLLOW_GIMBAL_YAW) && // 切换到舵跟随云台模式
			 chassis_move_transit->chassis_motor_mode == RUDDER_VECTOR_FOLLOW_GIMBAL_YAW) {
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		// 速度值清零
		chassis_move_transit->wz_set = 0.0f;
		chassis_move_transit->vx_set = 0.0f;
		chassis_move_transit->vy_set = 0.0f;
		// 更正轮电机转向
		chassis_move_transit->Forward_L.Judge_Speed_Direction = chassis_move_transit->Forward_R.Judge_Speed_Direction =
			chassis_move_transit->Back_L.Judge_Speed_Direction = chassis_move_transit->Back_R.Judge_Speed_Direction = -1;
	}
	chassis_move_transit->last_chassis_motor_mode = chassis_move_transit->chassis_motor_mode;
}
/**
 * @brief          底盘输入控制量设置及初步解算
 * @author         LYH 1.0
 * @param[in]      chassis_move_control：底盘数据指针
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t* chassis_move_control) {
	if (chassis_move_control == NULL) {
		return;
	}
	//期望机体速度
	float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	float relative_angle = 0.0f;
	float angle_error = 0.0f;
	//使用指针从CAN缓存读取期望值
	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
	if (chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) {// 底盘跟随云台模式
		float sin_yaw, cos_yaw = 0.0f;

		if (chassis_move_control->chassis_mode_CANsend == FOLLOW_GIMBAL_YAW_SLANT || chassis_move_control->chassis_mode_CANsend == RUDDER_FOLLOW_GIMBAL_YAW_SLANT) {
			//斜角跟随
			if (chassis_move_control->gimbal_data.relative_angle < (0.75 * PI) && chassis_move_control->gimbal_data.relative_angle>-0.25 * PI)
				chassis_move_control->chassis_relative_angle_set = 0.25 * PI;
			else
				chassis_move_control->chassis_relative_angle_set = -0.75 * PI;
		}
		else {
			//正向跟随
			if (chassis_move_control->gimbal_data.relative_angle >= HALF_PI)
				chassis_move_control->chassis_relative_angle_set = PI;
			else if (chassis_move_control->gimbal_data.relative_angle < -HALF_PI)
				chassis_move_control->chassis_relative_angle_set = -PI;
			else
				chassis_move_control->chassis_relative_angle_set = 0.0f;
		}
		relative_angle = chassis_move_control->gimbal_data.relative_angle;
		relative_angle = rad_format(relative_angle);

		sin_yaw = arm_sin_f32((relative_angle));
		cos_yaw = arm_cos_f32((relative_angle));

		//将云台系速度旋转至底盘系（底盘系和云台系相对角为0时，x=-y',y=x）
		chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
		chassis_move_control->vy_set = -1.0f * sin_yaw * vx_set + cos_yaw * vy_set;
		angle_error = rad_format(chassis_move_control->chassis_relative_angle_set - relative_angle);

		//底盘跟随死区设定为0.1
		if (fabsf(angle_error) < 0.1f) {
			chassis_move_control->wz_set = 0;
		}
		else {
			chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid, 0.0f, angle_error);
		}
	}

	else if (chassis_move_control->chassis_motor_mode == RUDDER_VECTOR_FOLLOW_GIMBAL_YAW) // 舵跟随云台模式(不旋转)
	{
		float sin_yaw, cos_yaw = 0.0f;
		chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);
		relative_angle = chassis_move_control->gimbal_data.relative_angle;

		relative_angle = rad_format(relative_angle);

		sin_yaw = arm_sin_f32((relative_angle));
		cos_yaw = arm_cos_f32((relative_angle));

		chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
		chassis_move_control->vy_set = -1.0f * sin_yaw * vx_set + cos_yaw * vy_set;
		chassis_move_control->wz_set = 0;
	}
	else if (chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_SPIN) // 小陀螺模式
	{
		float sin_yaw = 0.0f, cos_yaw = 0.0f;
		// static float temp_wz = 25.0f;
		static float temp_wz = 15.0f;
		// static float temp_wz = PI;
		relative_angle = chassis_move_control->gimbal_data.relative_angle;

		relative_angle = rad_format(relative_angle);

		//		//小陀螺角度补偿
		//	  relative_angle += 0.007f;
		relative_angle += chassis_move_control->wz * 0.005f;

		sin_yaw = arm_sin_f32((relative_angle));
		cos_yaw = arm_cos_f32((relative_angle));

		chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
		chassis_move_control->vy_set = -1.0f * sin_yaw * vx_set + cos_yaw * vy_set;
		chassis_move_control->chassis_relative_angle_set = rad_format(0.0);
		chassis_move_control->wz_set = -temp_wz;

		if (fabs(chassis_move_control->vx_set_CANsend) > 1.0f) {
			//			chassis_move_control->wz_set *= 0.1f;
			chassis_move_control->vx_set *= 0.4f;
			chassis_move_control->vy_set *= 0.4f;
			// chassis_move_control->wz_set *= 2.0f;//0.2
			chassis_move_control->wz_set *= 2.0f;//0.2
		}
		else if (fabs(chassis_move_control->vy_set_CANsend) > 1.0f) {
			//			chassis_move_control->wz_set *= 0.1f;
			chassis_move_control->vx_set *= 0.3f;
			chassis_move_control->vy_set *= 0.3f;
			// chassis_move_control->wz_set *= 2.0f;//0.2
			chassis_move_control->wz_set *= 2.5f;//0.2
		}
		else // 当原地时，加大转速

		{
			// chassis_move_control->wz_set *= 2.5f;//3.0
			chassis_move_control->wz_set *= 2.5f;//3.0
			// chassis_move_control->wz_set *= 3.0f;//3.0
		}
	}
	else if (chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW) {
		//不进行底盘跟随
		chassis_move_control->vx_set = vx_set;
		chassis_move_control->vy_set = vy_set;
		chassis_move_control->wz_set = angle_set;
	}
	else if (chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_RAW) {
		chassis_move_control->vx_set = vx_set;
		chassis_move_control->vy_set = vy_set;
		chassis_move_control->wz_set = angle_set;
	}


	//解算出轮速与舵角
	chassic_rudder_preliminary_A_S_solution(chassis_move_control);
}

/**
 * @brief          轮，舵初步角度速度控制量解算
 * @author         XQL  1.0
 *                 LYH  2.0
 * @param[in]      chassic_rudder_preliminary_solution：底盘数据指针
 * @note         	进行了略微的重构
 * @retval			void
 */
static void chassic_rudder_preliminary_A_S_solution(chassis_move_t* chassic_rudder_preliminary_solution) {
	float vx_set, vy_set, vw_set = 0.0f;
	const float sin_cos_45 = 0.70710678118f;
	float rotational_speed;
	float vx_plus, vx_minus, vy_plus, vy_minus;
	vx_set = chassic_rudder_preliminary_solution->vx_set;
	vy_set = chassic_rudder_preliminary_solution->vy_set;
	vw_set = chassic_rudder_preliminary_solution->wz_set;
	rotational_speed = vw_set * sin_cos_45;
	vx_plus = vx_set + rotational_speed;
	vx_minus = vx_set - rotational_speed;
	vy_plus = vy_set + rotational_speed;
	vy_minus = vy_set - rotational_speed;
	// 根据设置速度求轮电机转速//线速度
	chassic_rudder_preliminary_solution->Forward_L.wheel_speed = sqrtf(vy_plus * vy_plus + vx_plus * vx_plus);
	chassic_rudder_preliminary_solution->Back_L.wheel_speed = sqrtf(vy_minus * vy_minus + vx_plus * vx_plus);
	chassic_rudder_preliminary_solution->Back_R.wheel_speed = sqrtf(vy_minus * vy_minus + vx_minus * vx_minus);
	chassic_rudder_preliminary_solution->Forward_R.wheel_speed = -sqrtf(vy_plus * vy_plus + vx_minus * vx_minus); // 10.19
	// 根据速度反三角函数求角度                                                                                                                              //逆时针旋转

	chassic_rudder_preliminary_solution->Forward_L.rudder_angle = atan2f(vy_plus, vx_plus); // 0  3
	chassic_rudder_preliminary_solution->Back_L.rudder_angle = atan2f(vy_minus, vx_plus);    // 1  2
	chassic_rudder_preliminary_solution->Back_R.rudder_angle = atan2f(vy_minus, vx_minus);
	chassic_rudder_preliminary_solution->Forward_R.rudder_angle = atan2f(vy_plus, vx_minus);

	// 求编码值变量
	chassic_rudder_preliminary_solution->Forward_L.ecd_add = chassic_rudder_preliminary_solution->Forward_L.rudder_angle / Motor_Ecd_to_Rad;
	chassic_rudder_preliminary_solution->Back_L.ecd_add = chassic_rudder_preliminary_solution->Back_L.rudder_angle / Motor_Ecd_to_Rad;
	chassic_rudder_preliminary_solution->Back_R.ecd_add = chassic_rudder_preliminary_solution->Back_R.rudder_angle / Motor_Ecd_to_Rad;
	chassic_rudder_preliminary_solution->Forward_R.ecd_add = chassic_rudder_preliminary_solution->Forward_R.rudder_angle / Motor_Ecd_to_Rad;
	// 数据更新
	chassic_rudder_preliminary_solution->Forward_L.last_ecd_add = chassic_rudder_preliminary_solution->Forward_L.ecd_add;
	chassic_rudder_preliminary_solution->Back_L.last_ecd_add = chassic_rudder_preliminary_solution->Back_L.ecd_add;
	chassic_rudder_preliminary_solution->Back_R.last_ecd_add = chassic_rudder_preliminary_solution->Back_R.ecd_add;
	chassic_rudder_preliminary_solution->Forward_R.last_ecd_add = chassic_rudder_preliminary_solution->Forward_R.ecd_add;
}

/**
 * @brief          轮电机速度设置
 * @param[in]      chassis_speed_set：底盘数据指针
 * @retval         none
 */
static void chassis_speed_control_set(chassis_move_t* chassis_speed_set) {
	//期望速度=解算速度*速度方向*余弦优化系数
	chassis_speed_set->motor_chassis[0].speed_set = chassis_speed_set->Forward_L.wheel_speed * chassis_speed_set->Forward_L.Judge_Speed_Direction * chassis_speed_set->Forward_L.Judge_Speed_cosk;
	chassis_speed_set->motor_chassis[1].speed_set = chassis_speed_set->Forward_R.wheel_speed * chassis_speed_set->Forward_R.Judge_Speed_Direction * chassis_speed_set->Forward_R.Judge_Speed_cosk;
	chassis_speed_set->motor_chassis[2].speed_set = chassis_speed_set->Back_L.wheel_speed * chassis_speed_set->Back_L.Judge_Speed_Direction * chassis_speed_set->Back_L.Judge_Speed_cosk;
	chassis_speed_set->motor_chassis[3].speed_set = chassis_speed_set->Back_R.wheel_speed * chassis_speed_set->Back_R.Judge_Speed_Direction * chassis_speed_set->Back_R.Judge_Speed_cosk;

	float max_vector = 0.f, vector_rate = 0.f;

	for (uint8_t i = 0; i < 4; i++) {
		if (max_vector < fabs(chassis_speed_set->motor_chassis[i].speed_set)) {
			max_vector = fabs(chassis_speed_set->motor_chassis[i].speed_set);
		}
	}
	// 限制最大速度
	if (max_vector > MAX_WHEEL_SPEED) {
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (uint8_t i = 0; i < 4; i++) {
			chassis_speed_set->motor_chassis[i].speed_set *= vector_rate;
		}
	}
}

/**
 * @brief          舵电机输出控制量设置
 * @param[in]      chassis_move_control_loop：底盘数据指针
 * @retval         none
 */
static void rudder_control_loop(chassis_move_t* rudder_move_control_loop) {
	Rudder_motor_relative_angle_control(&rudder_move_control_loop->Forward_L);
	Rudder_motor_relative_angle_control(&rudder_move_control_loop->Back_L);
	Rudder_motor_relative_angle_control(&rudder_move_control_loop->Back_R);
	Rudder_motor_relative_angle_control(&rudder_move_control_loop->Forward_R);
}

/**
 * @brief          舵电机控制量设置
 * @author         LYH  1.0
 * @param[in]      chassis_motor：舵电机数据指针
 * @retval         none
 */
static void Rudder_motor_relative_angle_control(Rudder_Motor_t* chassis_motor) {
	if (chassis_motor == NULL || chassis_motor->rudder_motor_measure == NULL) {
		return;
	}
	float angle;
	// 计算目标位置
	if (fabsf(chassis_move.vx_set_CANsend) < 1.0f && fabsf(chassis_move.vy_set_CANsend) <= 1.0f && fabs(chassis_move.wz_set) < 1.0f) {
		//静止时，设定编码值为当前角度(防止鬼畜耗功率)
		chassis_motor->ecd_set = chassis_motor->rudder_motor_measure->ecd;
	}
	else {
		//开始舵轮控制
		//过圈处理
		if (chassis_motor->ecd_add > 0) {
			if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add > 8191) {
				chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add - 8191;
			}
			else if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add < 8191) {
				chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add;
			}
		}
		else if (chassis_motor->ecd_add < 0) {
			if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add < 0) {
				chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add + 8191;
			}
			else if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add > 0) {
				chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add;
			}
		}
		else if (chassis_motor->ecd_add == 0.0f && chassis_move.chassis_motor_mode != CHASSIS_VECTOR_SPIN) {
			chassis_motor->ecd_set = chassis_motor->rudder_motor_measure->ecd;
		}
		else {
			//chassis_motor->ecd_set =chassis_motor->ecd_zero_set;
			chassis_motor->ecd_set = chassis_motor->rudder_motor_measure->ecd;
		}
	}
	//求误差
	chassis_motor->ecd_error = chassis_motor->ecd_set - chassis_motor->rudder_motor_measure->ecd;

	// 就近原则：转劣弧
	if (chassis_motor->ecd_error > 4096) {
		chassis_motor->ecd_error = chassis_motor->ecd_error - 8192;
	}
	else if (chassis_motor->ecd_error < -4096) {
		chassis_motor->ecd_error = 8192 + chassis_motor->ecd_error;
	}

	//最优角：必要时反转而不是打舵(当误差大于四分之一圈时，去相反的方向)
	if (chassis_motor->ecd_error > 2048) {
		chassis_motor->ecd_error -= 4096;
		chassis_motor->Judge_Speed_Direction = -1;
	}
	else if (chassis_motor->ecd_error <= -2048) {
		chassis_motor->ecd_error += 4096;
		chassis_motor->Judge_Speed_Direction = -1;
	}
	else {
		chassis_motor->Judge_Speed_Direction = 1;
	}

	// 计算根据舵角误差，计算cosK,以削减轮电机转速，cosK的3次方效果较好
	angle = (chassis_motor->ecd_error) * Motor_Ecd_to_Rad;
	if (fabs(angle) > HALF_PI)
		angle = HALF_PI;
	else if (fabs(angle) < 0.05f && chassis_move.chassis_motor_mode != CHASSIS_VECTOR_SPIN)
		angle = 0; //防止因极小的误差导致四个轮子转速不一样
	const float cos_angle = arm_cos_f32(angle);
	chassis_motor->Judge_Speed_cosk = cos_angle * cos_angle * cos_angle;

	RUDDER_MOTOR_PID_CONTROL(chassis_motor);
}

/**
 * @brief          舵电机电流控制量计算
 * @param[in]      rudder_motor：舵电机数据指针
 * @retval         none
 */
static void RUDDER_MOTOR_PID_CONTROL(Rudder_Motor_t* rudder_motor) {
	if (rudder_motor == NULL) {
		return;
	}
	Matlab_PID_Calc(rudder_motor->ecd_error, 0, rudder_motor->motor_speed, &(rudder_motor->rudder_control));
	rudder_motor->given_current = rudder_motor->rudder_control.rudder_out.Out1;
}

/**
 * @brief          舵电机功率控制
 * @author         LYH  1.0
 * @param[in]      chassis_motor：舵电机数据指针
 * @retval         none
 */
static void RUDDER_POWER_CONTROL(chassis_move_t* rudder_power) {
	float scaled_m6020_power[4];//削减后的功率
	float rudder_cmd_power[4];//每个电机的功率
	rudder_power->rudder_power_control.rudder_.sumPowerCmd = 0;

	if (referee.game_robot_state_.chassis_power_limit > 55)
		rudder_power->rudder_power_control.rudder_.alloctablePower = 60 + (referee.power_heat_data_.buffer_energy - 30);
	else
		rudder_power->rudder_power_control.rudder_.alloctablePower = 60;//robot_state.chassis_power_limit+(power_heat_data_t.chassis_power_buffer-30);

	// 电流数值初始化
	rudder_power->rudder_given_current[0] = rudder_power->Forward_L.given_current;
	rudder_power->rudder_given_current[1] = rudder_power->Forward_R.given_current;
	rudder_power->rudder_given_current[2] = rudder_power->Back_L.given_current;
	rudder_power->rudder_given_current[3] = rudder_power->Back_R.given_current;
	for (int i = 0;i < 4;i++) {
		if (i == 1 || i == 2) {
			// 力矩电流
			rudder_power->rudder_torque_current[i] = rudder_power->rudder_given_current[i] * GM6020_Current_To_Out;
			// 功率模型预测
			rudder_cmd_power[i] = power_forecast(GM6020_K0, GM6020_K1, GM6020_K2, GM6020_K3, rudder_power->rudder_torque_current[i], rudder_power->rudder_omega[i]);
			// 负功率跳出
			if (rudder_power->rudder_power_control.forecast_motor_power[i] < 0) continue;
			// 电机功率累加
			rudder_power->rudder_power_control.rudder_.sumPowerCmd += rudder_cmd_power[i];
		}
	}
	//舵电机功率控制
	if (rudder_power->rudder_power_control.rudder_.sumPowerCmd > rudder_power->rudder_power_control.rudder_.alloctablePower) {
		for (int i = 0;i < 4;i++) {
			if (i == 1 || i == 2) {
				//舵电机衰减系数
				rudder_power->rudder_power_control.rudder_.power_scale[i] = rudder_power->rudder_power_control.rudder_.alloctablePower / rudder_power->rudder_power_control.rudder_.sumPowerCmd;
				//单个电机最终功率
				scaled_m6020_power[i] = rudder_cmd_power[i] * rudder_power->rudder_power_control.rudder_.power_scale[i];
				//总可用功率
				rudder_power->rudder_power_control.rudder_.alloctableSumPower += scaled_m6020_power[i];
				//反解电流
				rudder_power->rudder_power_control.MAX_current[i] = Power_reso_GM6020(GM6020_K0, GM6020_K1, GM6020_K2, GM6020_K3, rudder_power->rudder_omega[i], scaled_m6020_power[i]);
				//输出电流赋值
				rudder_power->rudder_given_current[i] = rudder_power->rudder_power_control.MAX_current[i];
			}
		}
	}
}

void CHASSIC_MOTOR_POWER_CONTROL(chassis_move_t* chassis_motor) {
	uint16_t wheel_power_limit = 40;
	float input_power = 0;		 // 输入功率(缓冲能量环)
	float scaled_motor_power[4];

	chassis_motor->power_control.POWER_MAX = referee.game_robot_state_.chassis_power_limit; //最终底盘的最大功率初始化
	chassis_motor->chassis_power_buffer = referee.power_heat_data_.buffer_energy;
	chassis_motor->power_control.forecast_total_power = 0; // 预测总功率初始化

	PID_Calc(&chassis_motor->buffer_pid, chassis_motor->chassis_power_buffer, 40); //使缓冲能量维持在一个稳定的范围

	wheel_power_limit = referee.game_robot_state_.chassis_power_limit;

	input_power = wheel_power_limit - chassis_motor->buffer_pid.out; //通过裁判系统的最大功率
	//将最小功率限制到45W防止掉线导致
	input_power = input_power < 45 ? 45 : input_power;
	chassis_motor->power_control.power_charge = input_power; //超级电容的最大充电功率

	if (chassis_motor->power_control.power_charge > 200)
		chassis_motor->power_control.power_charge = 200; //参考超电控制板允许的最大充电功率

	if (get_cap.cap_volt > 3.5f&&referee.power_heat_data_.buffer_energy>20) {
		if ((get_cap.cap_volt > 16.0f && chassis_move.key_shift == 2&&chassis_move.chassis_mode_CANsend!=SPIN)) //大超电，用于飞坡，冲刺
		{
			chassis_motor->power_control.POWER_MAX = 400;
		}
		else if (chassis_move.key_shift == 1)//小超电，用于上坡，平地加速
		{
			if (get_cap.cap_volt > 16) {
				chassis_motor->power_control.POWER_MAX = input_power + 50;
			}
			else if (get_cap.cap_volt > 13 && get_cap.cap_volt < 16) {
				chassis_motor->power_control.POWER_MAX = input_power + 35;
			}
			else {
				chassis_motor->power_control.POWER_MAX = input_power + 15;
			}
		}
		else//被动超电
		{
			if (referee.game_robot_state_.chassis_power_limit > 55)//被动超电（限制高等级浪费）
			{
				if (get_cap.cap_volt > 18)
					chassis_motor->power_control.POWER_MAX = 75;
				else
					chassis_motor->power_control.POWER_MAX = 50;
			}
			else if (referee.game_robot_state_.chassis_power_limit < 40)//被动超电（补偿虚弱）
			{
				if (get_cap.cap_volt > 18)
					chassis_motor->power_control.POWER_MAX = 50;
				else
					chassis_motor->power_control.POWER_MAX = 45;
			}
			else {
				if (get_cap.cap_volt > 22)//被动超电（初始状态）
					chassis_motor->power_control.POWER_MAX = input_power + 25;
				else if (get_cap.cap_volt > 20 && get_cap.cap_volt < 22)
					chassis_motor->power_control.POWER_MAX = input_power + 20;
				else if (get_cap.cap_volt > 18 && get_cap.cap_volt < 20)
					chassis_motor->power_control.POWER_MAX = input_power + 10;
				else
					chassis_motor->power_control.POWER_MAX = input_power - 5;
			}
		}
	}
	else {
		chassis_motor->power_control.POWER_MAX = input_power;
	}

	for (uint8_t i = 0; i < 4; i++) // 获得所有3508电机的功率和总功率
	{
		chassis_motor->power_control.forecast_motor_power[i] =
			chassis_motor->motor_chassis[i].give_current * toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm +
			k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm +
			k2 * chassis_motor->motor_chassis[i].give_current * chassis_motor->motor_chassis[i].give_current + constant_3508;

		if (chassis_motor->power_control.forecast_motor_power[i] < 0)  	continue; // 忽略负电

		if (i == 1 || i == 2)
			chassis_motor->power_control.forecast_total_power += chassis_motor->power_control.forecast_motor_power[i];
	}

	if (chassis_motor->power_control.forecast_total_power > chassis_motor->power_control.POWER_MAX) // 超功率模型衰减
	{
		float power_scale = chassis_motor->power_control.POWER_MAX / chassis_motor->power_control.forecast_total_power;

		for (uint8_t i = 0; i < 4; i++) {
			scaled_motor_power[i] = chassis_motor->power_control.forecast_motor_power[i] * power_scale; // 获得衰减后的功率
			if (scaled_motor_power[i] < 0)		continue;
		}
		for (uint8_t i = 0; i < 4; i++) {
			float b = toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm;
			float c = k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_motor_power[i] + constant_3508;

			if (chassis_motor->motor_chassis[i].give_current > 0)  //避免超过最大电流
			{
				chassis_motor->power_control.MAX_current[i] = (-b + sqrt(b * b - 4 * k2 * c)) / (2 * k2);
				if (chassis_motor->power_control.MAX_current[i] > MAX_MOTOR_CAN_CURRENT) {
					chassis_motor->motor_chassis[i].give_current = MAX_MOTOR_CAN_CURRENT;
				}
				else
					chassis_motor->motor_chassis[i].give_current = chassis_motor->power_control.MAX_current[i];
			}
			else {
				chassis_motor->power_control.MAX_current[i] = (-b - sqrt(b * b - 4 * k2 * c)) / (2 * k2);
				if (chassis_motor->power_control.MAX_current[i] < -MAX_MOTOR_CAN_CURRENT) {
					chassis_motor->motor_chassis[i].give_current = -MAX_MOTOR_CAN_CURRENT;
				}
				else
					chassis_motor->motor_chassis[i].give_current = chassis_motor->power_control.MAX_current[i];
			}
		}
	}
}


/**
 * @brief          轮电机电流控制量计算
 * @author         LYH
 * @param[in]      chassis_motor：轮电机数据指针
 * @retval         none
 */
static void CHASSIC_MOTOR_PID_CONTROL(chassis_move_t* chassis_motor) {
	chassis_speed_control_set(chassis_motor);

	for (uint8_t i = 0; i < 4; i++) {
		chassis_motor->motor_chassis[i].give_current = PID_Calc(&chassis_motor->motor_chassis[i].chassis_pid,
																chassis_motor->motor_chassis[i].speed, chassis_motor->motor_chassis[i].speed_set);
		//给轮功率限制留余地，保留一定的功率额度
		if (fabs(chassis_motor->power_control.speed[i]) < chassis_motor->power_control.SPEED_MIN) {
			chassis_motor->power_control.speed[i] = chassis_motor->power_control.SPEED_MIN;
		}
	}

	CHASSIC_MOTOR_POWER_CONTROL(chassis_motor);
}

/**
  * @brief          舵电机PID参初始化
  * @author         XQL 1.0
							LYH 2.0
  * @param[in]      rudder_init：底盘数据指针
  * @param[in]      PID[8]：PID参数
  * @retval         none
  */
static void RUDDER_PID_INIT(chassis_move_t* rudder_init, const float PID[8]) {
	rudder_init->Forward_L.rudder_control.rudder_in.P_P = rudder_init->Forward_R.rudder_control.rudder_in.P_P = rudder_init->Back_L.rudder_control.rudder_in.P_P = rudder_init->Back_R.rudder_control.rudder_in.P_P = PID[0];

	rudder_init->Forward_L.rudder_control.rudder_in.P_I = rudder_init->Forward_R.rudder_control.rudder_in.P_I = rudder_init->Back_L.rudder_control.rudder_in.P_I = rudder_init->Back_R.rudder_control.rudder_in.P_I = PID[1];

	rudder_init->Forward_L.rudder_control.rudder_in.P_D = rudder_init->Forward_R.rudder_control.rudder_in.P_D = rudder_init->Back_L.rudder_control.rudder_in.P_D = rudder_init->Back_R.rudder_control.rudder_in.P_D = PID[2];

	rudder_init->Forward_L.rudder_control.rudder_in.P_N = rudder_init->Forward_R.rudder_control.rudder_in.P_N = rudder_init->Back_L.rudder_control.rudder_in.P_N = rudder_init->Back_R.rudder_control.rudder_in.P_N = PID[3];

	rudder_init->Forward_L.rudder_control.rudder_in.S_P = rudder_init->Forward_R.rudder_control.rudder_in.S_P = rudder_init->Back_L.rudder_control.rudder_in.S_P = rudder_init->Back_R.rudder_control.rudder_in.S_P = PID[4];

	rudder_init->Forward_L.rudder_control.rudder_in.S_I = rudder_init->Forward_R.rudder_control.rudder_in.S_I = rudder_init->Back_L.rudder_control.rudder_in.S_I = rudder_init->Back_R.rudder_control.rudder_in.S_I = PID[5];

	rudder_init->Forward_L.rudder_control.rudder_in.S_D = rudder_init->Forward_R.rudder_control.rudder_in.S_D = rudder_init->Back_L.rudder_control.rudder_in.S_D = rudder_init->Back_R.rudder_control.rudder_in.S_D = PID[6];

	rudder_init->Forward_L.rudder_control.rudder_in.S_N = rudder_init->Forward_R.rudder_control.rudder_in.S_N = rudder_init->Back_L.rudder_control.rudder_in.S_N = rudder_init->Back_R.rudder_control.rudder_in.S_N = PID[7];
}

/**
 * @brief          速度输入量设置
 * @author         LYH
 * @param[in]      *vx_set：x方向速度设置量
 * @param[in]      *vy_set：y方向速度设置量
 * @param[in]      chassis_move_rc_to_vector：底盘数据指针
 * @retval         none
 */
void chassis_rc_to_control_vector(float* vx_set, float* vy_set, chassis_move_t* chassis_move_rc_to_vector) {
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) {
		return;
	}
	kx = 1.5f;
	ky = 1.5f;
	kw = 0.5;//1.0f;

	//对接收的数据进行映射
	chassis_move_rc_to_vector->vx_set_CANsend = chassis_move_rc_to_vector->vx_temp * 0.002f* MAX_WHEEL_SPEED;
	//设置死区
	if (fabsf(chassis_move_rc_to_vector->vx_set_CANsend) <= 5.0f) {
		chassis_move_rc_to_vector->vx_set_CANsend = 0;
	}
	chassis_move_rc_to_vector->vy_set_CANsend = chassis_move_rc_to_vector->vy_temp * 0.002f * MAX_WHEEL_SPEED;
	if (fabsf(chassis_move_rc_to_vector->vy_set_CANsend) <= 5.0f) {
		chassis_move_rc_to_vector->vy_set_CANsend = 0;
	}

	//斜坡函数处理
	chassis_move_rc_to_vector->Slope_X_Speed.Target = kx * (-chassis_move_rc_to_vector->vx_set_CANsend);
	chassis_move_rc_to_vector->Slope_X_Speed.Now_Real = -chassis_move_rc_to_vector->vx;

	chassis_move_rc_to_vector->Slope_Y_Speed.Target = ky * (-chassis_move_rc_to_vector->vy_set_CANsend);
	chassis_move_rc_to_vector->Slope_Y_Speed.Now_Real = -chassis_move_rc_to_vector->vy;

	slope_calc(&chassis_move_rc_to_vector->Slope_X_Speed);
	slope_calc(&chassis_move_rc_to_vector->Slope_Y_Speed);

	*vx_set = chassis_move_rc_to_vector->Slope_X_Speed.Out;
	*vy_set = chassis_move_rc_to_vector->Slope_Y_Speed.Out;
	// *vx_set = - chassis_move_rc_to_vector->vx_set_CANsend;
	// *vy_set = - chassis_move_rc_to_vector->vy_set_CANsend;
}

static void chassis_Self_Resolution(chassis_move_t* chassis_Self_Res) {
	// 根据电机编码器与陀螺仪计算速度和角度
	chassis_Self_Res->wz = 0.0f;
	static float tmp_now_Omega[4] = { 0 };
	uint8_t i = 0;
	for (i = 0; i < 4; i++) {
		tmp_now_Omega[i] = chassis_Self_Res->motor_chassis[i].speed * 19 / 268.0f / 17.0f;
	}
	chassis_Self_Res->wz =
		((tmp_now_Omega[1] * arm_sin_f32(DEG2R(chassis_Self_Res->Forward_R.rudder_angle) - Wheel_Azimuth[3]) * Wheel_Radius / Wheel_To_Core_Distance[1]) / 2.0f)
		+ ((tmp_now_Omega[2] * arm_sin_f32(DEG2R(chassis_Self_Res->Back_L.rudder_angle) - Wheel_Azimuth[1]) * Wheel_Radius / Wheel_To_Core_Distance[2]) / 2.0f);
}

void UI_task(void const* argument) {
	for (;;) {
		uint16_t robot_id = referee.game_robot_state_.robot_id;
		if (!(robot_id > 0)) {
			osDelay(100);
			continue;
		}

		if (ui_init == false || rece_.ui_refresh_flag) {
			UI_Draw_String(&UI.UI_String[0].String, (char*)"110", UI_Graph_ADD, 0,
						   UI_Color_Purplish_red, 20, 22, 2, 740, 874,
						   (char*)"SHOOT  FRIC  SPIN  CAP");
			UI_PushUp_String(&UI.UI_String[0], robot_id);
			osDelay(80);
			//"BULLET"字符
			UI_Draw_String(&UI.UI_String[1].String, (char*)"111", UI_Graph_ADD, 0,
						   UI_Color_Main, 20, 6, 2, 129, 753, (char*)"BULLET");
			UI_PushUp_String(&UI.UI_String[1], robot_id);
			osDelay(80);
			//"CAP"字符
			UI_Draw_String(&UI.UI_String[2].String, (char*)"112", UI_Graph_ADD, 0,
						   UI_Color_Yellow, 20, 3, 2, 1454, 800, (char*)"CAP");
			UI_PushUp_String(&UI.UI_String[2], robot_id);
			osDelay(80);
			//RFID字体
			UI_Draw_String(&UI.UI_String[3].String, (char*)"113", UI_Graph_ADD, 0,
						   UI_Color_Main, 20, 4, 2, 129, 683, (char*)"RFID");
			UI_PushUp_String(&UI.UI_String[3], robot_id);
			osDelay(80);
			//方位圆
			UI_Draw_Circle(&UI.UI_Graph1[0].Graphic[0], (char*)"113", UI_Graph_ADD, 2,
						   UI_Color_Green, 4, 737, 124, 70);
			UI_PushUp_Graphs(1, &UI.UI_Graph1[0], robot_id);
			osDelay(80);
			UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[0], (char*)"100", UI_Graph_ADD,
							  1, UI_Color_Green, 5, 872, 840, 963, 885);
			UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[1], (char*)"101", UI_Graph_ADD,
							  1, UI_Color_Green, 5, 991, 840, 1080, 885);
			UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[2], (char*)"102", UI_Graph_ADD,
							  1, UI_Color_Green, 5, 1103, 840, 1192, 885);
			UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[3], (char*)"103", UI_Graph_ADD,
							  1, UI_Color_Green, 5, 731, 840, 842, 885);
			UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[4], (char*)"104", UI_Graph_ADD,
							  1, UI_Color_Orange, 5, 114, 720, 257, 760);
			UI_PushUp_Graphs(5, &UI.UI_Graph5[0], robot_id);
			osDelay(80);

			UI_Draw_Line(&UI.UI_Graph7[0].Graphic[0], (char*)"120", UI_Graph_ADD, 0,
						 UI_Color_Yellow, 2, 900, 510, 1020, 510);
			UI_Draw_Line(&UI.UI_Graph7[0].Graphic[1], (char*)"121", UI_Graph_ADD, 0,
						 UI_Color_Yellow, 2, 940, 480, 980, 480);
			UI_Draw_Line(&UI.UI_Graph7[0].Graphic[2], (char*)"122", UI_Graph_ADD, 0,
						 UI_Color_Yellow, 2, 960, 540, 960, 450);
			UI_Draw_Line(&UI.UI_Graph7[0].Graphic[3], (char*)"123", UI_Graph_ADD, 0,
						 UI_Color_Yellow, 2, 960, 540, 960, 540);
			UI_Draw_Line(&UI.UI_Graph7[0].Graphic[4], (char*)"124", UI_Graph_ADD, 0,
						 UI_Color_Pink, 30, 1240, 40, 1240, 40);
			UI_Draw_Line(&UI.UI_Graph7[0].Graphic[5], (char*)"125", UI_Graph_ADD, 0,
						 UI_Color_Cyan, 30, 1300, 40, 1300, 40);
			UI_Draw_Rectangle(&UI.UI_Graph7[0].Graphic[6], (char*)"126", UI_Graph_ADD,
							  1, UI_Color_White, 3, 557, 289, 1355, 769);
			UI_PushUp_Graphs(7, &UI.UI_Graph7[0], robot_id);
			osDelay(80);
			UI_Draw_Line(&UI.UI_Graph2[0].Graphic[0], (char*)"130", UI_Graph_ADD, 3,
						 UI_Color_Purplish_red, 4, 667, 124, 807, 124);
			UI_Draw_Line(&UI.UI_Graph2[0].Graphic[1], (char*)"131", UI_Graph_ADD, 3,
						 UI_Color_Pink, 4, 737, 124, 737, 194);
			UI_PushUp_Graphs(2, &UI.UI_Graph2[0], robot_id);
			osDelay(80);
			UI_Draw_Line(&UI.UI_Graph1[1].Graphic[0], (char*)"132", UI_Graph_ADD, 3,
						 UI_Color_Green, 30, 1540, 790, 1540, 790);
			UI_PushUp_Graphs(1, &UI.UI_Graph1[1], robot_id);
			osDelay(80);
			ui_init = true;
		}
		else {
			int16_t cap_volt = 1540 + get_cap.cap_energy / 6.f;
			UI_Draw_Line(&UI.UI_Graph1[1].Graphic[0], (char*)"132", UI_Graph_Change, 3,
						 UI_Color_Green, 30, 1540, 790, cap_volt, 790);
			UI_PushUp_Graphs(1, &UI.UI_Graph1[1], robot_id);
			osDelay(80);
			// int16_t left_len = 40 ;
			// int16_t right_len = 40;
			// UI_Draw_Line(&UI.UI_Graph7[0].Graphic[4], (char*)"124", UI_Graph_Change, 0,
			// 			 UI_Color_Pink, 30, 1240, 40, 1240, left_len);
			// UI_Draw_Line(&UI.UI_Graph7[0].Graphic[5], (char*)"125", UI_Graph_Change, 0,
			// 			 UI_Color_Cyan, 30, 1300, 40, 1300, right_len);
			if (rece_.aim_flag) {
				UI_Draw_Rectangle(&UI.UI_Graph7[0].Graphic[6], (char*)"126", UI_Graph_Change,
								  1, UI_Color_Green, 3, 557, 289, 1355, 769);
			}
			else {
				UI_Draw_Rectangle(&UI.UI_Graph7[0].Graphic[6], (char*)"126", UI_Graph_Change,
								  1, UI_Color_Orange, 3, 557, 289, 1355, 769);
			}
			UI_PushUp_Graphs(7, &UI.UI_Graph7[0], robot_id);
			osDelay(80);
			if (rece_.fric_flag&&referee.game_robot_state_.power_management_shooter_output) {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[0], (char*)"100", UI_Graph_ADD,
								  1, UI_Color_Green, 5, 872, 840, 963, 885);
			}
			else {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[0], (char*)"100", UI_Graph_Del,
								  1, UI_Color_Green, 5, 872, 840, 963, 885);
			}
			if (rece_.cap_flag) {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[2], (char*)"102", UI_Graph_ADD,
								  1, UI_Color_Green, 5, 1103, 840, 1192, 885);
			}
			else {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[2], (char*)"102", UI_Graph_Del,
								  1, UI_Color_Green, 5, 1103, 840, 1192, 885);
			}
			// if (referee.rfid_status_.rfid_status > 0) {
			if (chassis_move.chassis_mode_CANsend == SPIN) {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[1], (char*)"101", UI_Graph_ADD,
								  1, UI_Color_Green, 5, 991, 840, 1080, 885);
			}
			else {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[1], (char*)"101", UI_Graph_Del,
								  1, UI_Color_Green, 5, 991, 840, 1080, 885);
			}
			if (rece_.shoot_flag) {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[3], (char*)"103",
								  UI_Graph_Change, 1, UI_Color_Purplish_red, 5, 731, 840,
								  842, 885);
			}
			else {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[3], (char*)"103",
								  UI_Graph_Change, 1, UI_Color_Green, 5, 731, 840, 842,
								  885);
			}
			if (referee.projectile_allowance.projectile_allowance_17mm <= 10) {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[4], (char*)"104", UI_Graph_ADD,
								  1, UI_Color_Orange, 5, 114, 720, 257, 760);
			}
			else {
				UI_Draw_Rectangle(&UI.UI_Graph5[0].Graphic[4], (char*)"104", UI_Graph_Del,
								  1, UI_Color_Orange, 5, 114, 720, 257, 760);
			}
			UI_PushUp_Graphs(5, &UI.UI_Graph5[0], robot_id);
			osDelay(80);
			int16_t sin_theta =
				arm_sin_f32(chassis_move.gimbal_data.relative_angle) * 70;
			int16_t cos_theta =
				arm_cos_f32(chassis_move.gimbal_data.relative_angle) * 70;
			int16_t sin_theta_90 =
				arm_sin_f32(chassis_move.gimbal_data.relative_angle + HALF_PI) *
				70;
			int16_t cos_theta_90 =
				arm_cos_f32(chassis_move.gimbal_data.relative_angle + HALF_PI) *
				70;
			UI_Draw_Line(&UI.UI_Graph2[0].Graphic[0], (char*)"130", UI_Graph_Change, 3,
						 UI_Color_Purplish_red, 4, 737 + sin_theta_90,
						 124 + cos_theta_90, 737 - sin_theta_90, 124 - cos_theta_90);
			UI_Draw_Line(&UI.UI_Graph2[0].Graphic[1], (char*)"131", UI_Graph_Change, 3,
						 UI_Color_Pink, 4, 737, 124, 737 + sin_theta, 124 + cos_theta);
			UI_PushUp_Graphs(2, &UI.UI_Graph2[0], robot_id);
			osDelay(80);
		}
	}
}
