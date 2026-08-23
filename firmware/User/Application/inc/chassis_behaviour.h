
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "main.h"
#include "chassis_task.h"

#define CHASSIS_OPEN_RC_SCALE 10 //在 chassis_open 模型下，遥控器乘以该比例发送到can上

#define GIMBAL_DISCONNECT                 0
#define FOLLOW_GIMBAL_YAW                 0x0B
#define RUDDER_FOLLOW_GIMBAL_YAW          0x09
#define SPIN                              0x05
#define NO_MOVE                           0x0A
#define ZERO                              0xAA
#define FOLLOW_GIMBAL_YAW_SLANT           0x0F   //底盘斜向跟随
#define RUDDER_FOLLOW_GIMBAL_YAW_SLANT    0x0D   //舵斜向跟随

typedef enum
{
  CHASSIS_ZERO_FORCE,                  //底盘无力
  CHASSIS_NO_MOVE,                     //底盘保持不动
  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //正常步兵底盘跟随云台
  CHASSIS_NO_FOLLOW_YAW,               //底盘不跟随角度
	CHASSIS_INFANTRY_SPIN,               //小陀螺
  CHASSIS_OPEN,                         //遥控器的值乘以比例直接发送到can总线上
		RUDDER_INFANTRY_FOLLOW_GIMBAL_YAW    //舵跟随云台模式
} chassis_behaviour_e;

extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
extern void chassis_behaviour_control_set(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);
extern chassis_behaviour_e chassis_behaviour_mode;
#endif
