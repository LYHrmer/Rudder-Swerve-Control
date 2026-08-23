/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#define CHASSIS_CAN hcan2
#define RUDDER_CAN hcan2
#define CAP_CAN hcan2
#define BOARD_CAN hcan1

#define CAN_FEEDBACK_FREAM_ID_A       0x222
#define CAN_FEEDBACK_FREAM_ID_B       0x223
#define CAN_CTRL_FREAM_ID             0x224       //CAN帧ID号
#define CAN_PITCHANGLE_ID  0x213

  //自研超电
typedef enum {
  CAN_CHASSIS_SEND_ID = 0x200,
  CAN_3508_M1_ID = 0x201,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,

  CAN_RUDDER_SEND_ID = 0x1FE,
  CAN_Forward_L_ID = 0x205,
  CAN_Forward_R_ID = 0x206,
  CAN_BACK_L_ID = 0x207,
  CAN_BACK_R_ID = 0x208,

  CAN_YAW_ID = 0x206,

  CAN_CAPID = 0x112,

  CAN_BOARD_COMM_ID = 0x115,
  CAN_REFEREE_ID = 0x114,
} can_msg_id_e;

#pragma pack(push,1)

typedef struct {
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} motor_measure_t;

typedef struct {
  uint16_t cap_energy;
  uint16_t input_power;
  uint16_t output_power;
  float cap_volt;
  uint8_t cap_state;
} cap_measure_t;


typedef struct {
  uint8_t cooling_heat;
  uint8_t heat_limit;
  uint16_t current_heat;
  int16_t spin_speed;
  uint8_t robot_id;

  uint8_t shooter_output : 1;
  uint8_t cap_online_flag : 1;
  uint8_t wheel_online_flag : 1;
  uint8_t rudder_online_flag : 1;
  uint8_t chassis_referee_online_flag : 1;
  uint8_t reserved : 3;
} send_pack;

typedef struct {
  int16_t x_speed;
  int16_t y_speed;
  uint8_t mode;
  uint8_t shoot_flag : 1;       //单连发
  uint8_t fric_flag : 1;        //摩擦轮
  uint8_t aim_flag : 2;         //自瞄
  uint8_t cap_flag : 1;         //超电
  uint8_t ready_flag : 1;       //启动
  uint8_t ui_refresh_flag : 1;  //ui刷新
  uint8_t reset_flag : 1;  //重启
  uint16_t reserved;
} rece_pack;

#pragma pack(pop)

extern cap_measure_t get_cap;

extern send_pack send_;
extern rece_pack rece_;

void CAN_CMD_cap(uint16_t target_power, uint16_t flag);
void CAN_cmd_rudder(int16_t forward_L, int16_t forward_R, int16_t back_L, int16_t back_R);
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

const motor_measure_t* get_Forward_L_motor_measure_point(void);
const motor_measure_t* get_Forward_R_motor_measure_point(void);
const motor_measure_t* get_Back_R_motor_measure_point(void);
const motor_measure_t* get_Back_L_motor_measure_point(void);

const motor_measure_t* get_chassis_motor_measure_point(uint8_t i);

cap_measure_t* get_cap_measure_point(void);

void CAN_board_send(uint16_t heat_limit, uint8_t cooling_value, uint16_t heat, float spin_speed, uint8_t robot_id, uint8_t shooter_output);

#endif
