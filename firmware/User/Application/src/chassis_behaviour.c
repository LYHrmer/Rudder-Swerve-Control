#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "arm_math.h"


/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(float* vx_can_set, float* vy_can_set, float* wz_can_set, chassis_move_t* chassis_move_rc_to_vector);


/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_move_control(float* vx_set, float* vy_set, float* wz_set, chassis_move_t* chassis_move_rc_to_vector);


/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_infantry_follow_gimbal_yaw_control(float* vx_set, float* vy_set, float* angle_set, chassis_move_t* chassis_move_rc_to_vector);
extern chassis_move_t chassis_move;
//底盘行为状态机
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
chassis_behaviour_e chassis_behaviour_mode_last = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;

/**
  * @brief          底盘控制模式设置
  * @author         LYH
  * @param[in]      chassis_move_mode：底盘数据
  * @retval         void
  */
void chassis_behaviour_mode_set(chassis_move_t* chassis_move_mode) {
    if (chassis_move_mode->chassis_mode_CANsend == FOLLOW_GIMBAL_YAW || chassis_move_mode->chassis_mode_CANsend == FOLLOW_GIMBAL_YAW_SLANT) {//底盘跟随云台，（前是正常跟随，后是斜角跟随）
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_move_mode->chassis_mode_CANsend == SPIN) {//小陀螺
        chassis_behaviour_mode = CHASSIS_INFANTRY_SPIN;
    }
    else if (chassis_move_mode->chassis_mode_CANsend == RUDDER_FOLLOW_GIMBAL_YAW || chassis_move_mode->chassis_mode_CANsend == RUDDER_FOLLOW_GIMBAL_YAW_SLANT) {//舵跟随云台(同上)
        chassis_behaviour_mode = RUDDER_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_move_mode->chassis_mode_CANsend == NO_MOVE) {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }
    else {//静止
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }

    //根据行为状态机选择底盘状态机
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) {
        chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_RAW; //当行为是底盘无力，则设置底盘状态机为 raw，原生状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE) {
        chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不移动，则设置底盘状态机为 底盘不跟随角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) {
        chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //当行为是正常步兵跟随云台，则设置底盘状态机为 底盘跟随云台角度 状态机。
    }
    else if (chassis_behaviour_mode == RUDDER_INFANTRY_FOLLOW_GIMBAL_YAW) {
        //			if(chassis_behaviour_mode_last ==CHASSIS_VECTOR_SPIN && fabs(chassis_move.gimbal_data.relative_angle)>0.1)
        //        chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //当行为是舵跟随云台，则设置底盘状态机为 舵跟随云台角度 状态机。
        //			else
        //				chassis_move_mode->chassis_motor_mode = RUDDER_VECTOR_FOLLOW_GIMBAL_YAW;
        if (chassis_behaviour_mode_last != CHASSIS_INFANTRY_SPIN && (fabs(chassis_move_mode->wz_set) == 0))
            chassis_move_mode->chassis_motor_mode = RUDDER_VECTOR_FOLLOW_GIMBAL_YAW;
        else
            chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_SPIN) {
        chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_SPIN; //小陀螺
    }
    /* debug */
    // chassis_behaviour_mode=chassis_behaviour_mode_last;
}

/**
  * @brief          底盘控制量设置
  * @author         XQL 1.0
  *                 LYH 2.0
  * @param[in]      *vx_set：x方向设定速度
  * @param[in]      *vy_set：y方向设定速度
  * @param[in]      *angle_set：设定角度
  * @param[in]      chassis_move_rc_to_vector：底盘数据
  * @retval         返回空
  */
void chassis_behaviour_control_set(float* vx_set, float* vy_set, float* angle_set, chassis_move_t* chassis_move_rc_to_vector) {
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    //急停(和急停相同，未来可以改成内环增加摩擦力防止移动)
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE) {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    //底盘跟随
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == RUDDER_INFANTRY_FOLLOW_GIMBAL_YAW) {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_SPIN) {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(float* vx_can_set, float* vy_can_set, float* wz_can_set, chassis_move_t* chassis_move_rc_to_vector) {
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}


/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_move_control(float* vx_set, float* vy_set, float* wz_set, chassis_move_t* chassis_move_rc_to_vector) {
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}


/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set：前进的速度
  * @param[in]      vy_set：左右的速度
  * @param[in]      angle_set：底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector：底盘数据
  * @retval         返回空
  */
static void chassis_infantry_follow_gimbal_yaw_control(float* vx_set, float* vy_set, float* angle_set, chassis_move_t* Chassis_move_rc_to_vector) {
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || Chassis_move_rc_to_vector == NULL) {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, Chassis_move_rc_to_vector);
    *angle_set = 0.0f;

}
