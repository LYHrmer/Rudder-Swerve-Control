/**
 ****************************(C) COPYRIGHT 2019 DJI****************************
 * @file       can_receive.c/h
 * @brief      there is CAN interrupt function  to receive motor data,
 *             and CAN send function to send motor current to control motor.
 *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
 * @note
 * @history
 */

#include "CAN_receive.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "chassis_task.h"
#include "CRC8_CRC16.h"
#include "UI_task.h"
#include "detect_task.h"
#include "string.h"
#include "arm_math.h"
#include "remote_control.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern chassis_move_t chassis_move;
extern int16_t key_ctrl;

#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

motor_measure_t motor_chassis[4];                         // 0 1 2 3
motor_measure_t Forward_L, Forward_R, Back_R, Back_L;     // 0 1 3 2

cap_measure_t get_cap;

rece_pack rece_;
send_pack send_;

static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

static CAN_TxHeaderTypeDef rudder_tx_message;
static uint8_t rudder_can_send_data[8];

static CAN_TxHeaderTypeDef capid_tx_message;
static uint8_t capid_can_send_data[8];

static CAN_TxHeaderTypeDef tx_header;
static uint8_t shoot_heat_can_send_data[8];

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @author         LYH
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */

float voltent = 0.0f, current = 0.0f;

//CAN消息接收中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {

  if (hcan == &hcan1) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId) {
    case CAN_YAW_ID://接收yaw电机数据
    {
      get_motor_measure(&chassis_move.yaw, rx_data);
      break;
    }
    case CAN_BOARD_COMM_ID:
    {
      memcpy(&rece_, rx_data, 8);
      chassis_move.vx_temp = rece_.x_speed;
      chassis_move.vy_temp = rece_.y_speed;
      chassis_move.chassis_mode_CANsend = rece_.mode;
      chassis_move.key_shift = rece_.cap_flag;
      chassis_move.ready_flag = rece_.ready_flag;
      break;
    }
    default:
    {
      break;
    }
    }
  }
  if (hcan == &hcan2) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId) {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
      static uint8_t i = 0;
      i = rx_header.StdId - CAN_3508_M1_ID;
      get_motor_measure(&motor_chassis[i], rx_data);
      switch (i) {
        case 1:
          Detect_Feed(DEV_Wheel_RF);
          break;
        case 2:
          Detect_Feed(DEV_Wheel_LB);
          break;
        default:
          break;
      }
      break;
    }
    case CAN_Forward_L_ID:
    {
      get_motor_measure(&Forward_L, rx_data);
      break;
    }
    case CAN_Forward_R_ID:
    {
      get_motor_measure(&Forward_R, rx_data);
      Detect_Feed(DEV_Rudder_RF);
      break;
    }
    case CAN_BACK_L_ID:
    {
      get_motor_measure(&Back_L, rx_data);
      Detect_Feed(DEV_Rudder_LB);
      break;
    }
    case CAN_BACK_R_ID:
    {
      get_motor_measure(&Back_R, rx_data);
      break;
    }
    case 0x111://自研超级电容数据
    {
      memcpy(&get_cap.cap_energy, rx_data, 2);
      memcpy(&get_cap.input_power, rx_data + 2, 2);
      memcpy(&get_cap.output_power, rx_data + 4, 2);
      get_cap.cap_volt = sqrtf(get_cap.cap_energy * 0.45454545f);
      Detect_Feed(DEV_CAP);
      break;
    }
    default:
    {
      break;
    }
    }
  }
}

/**
 * @brief          发送舵电机控制电流(0x205,0x206,0x207,0x208)
 * @author         LYH
 * @param[in]      forward_L: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      forward_R: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      back_L: (0x207) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      back_R: (0x208) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
void CAN_cmd_rudder(int16_t forward_L, int16_t forward_R, int16_t back_L, int16_t back_R) {
  uint32_t send_mail_box;
  rudder_tx_message.StdId = CAN_RUDDER_SEND_ID;
  rudder_tx_message.IDE = CAN_ID_STD;
  rudder_tx_message.RTR = CAN_RTR_DATA;
  rudder_tx_message.DLC = 0x08;
  rudder_can_send_data[0] = 0;
  rudder_can_send_data[1] = 0;
  rudder_can_send_data[2] = (forward_R >> 8);
  rudder_can_send_data[3] = forward_R;
  rudder_can_send_data[4] = (back_L >> 8);
  rudder_can_send_data[5] = back_L;
  rudder_can_send_data[6] = 0;
  rudder_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&RUDDER_CAN, &rudder_tx_message, rudder_can_send_data, &send_mail_box);
}

/**
 * @brief          发送轮电机控制电流(0x201,0x202,0x203,0x204)
 * @author         LYH
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_SEND_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


//发送裁判系统数据，包括射击热量上限，当前热量，冷却，弹速
void CAN_board_send(uint16_t heat_limit, uint8_t cooling_value, uint16_t heat, float spin_speed, uint8_t robot_id, uint8_t shooter_output) {
  uint32_t send_mail_box;
  uint16_t tmp_speed;
  uint8_t tx_data[8] = { 0 };
  tx_header.StdId = CAN_REFEREE_ID;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 0x08;
  tmp_speed = (uint16_t)(spin_speed * 100);//扩大速度精准度
  send_.heat_limit = heat_limit / 10u;
  send_.cooling_heat = cooling_value;
  send_.current_heat = heat;
  send_.spin_speed = tmp_speed;
  send_.robot_id = robot_id;
  send_.shooter_output = shooter_output;
  send_.cap_online_flag = Detect_IsOnline(DEV_CAP);
  send_.chassis_referee_online_flag = Detect_IsOnline(DEV_Referee);
  send_.wheel_online_flag = Detect_IsOnline(DEV_Wheel_LB) && Detect_IsOnline(DEV_Wheel_RF);
  send_.rudder_online_flag = Detect_IsOnline(DEV_Rudder_LB) && Detect_IsOnline(DEV_Rudder_RF);
  memcpy(tx_data, &send_, sizeof(send_));

  HAL_CAN_AddTxMessage(&BOARD_CAN, &tx_header, tx_data, &send_mail_box);
}

/**
 * @brief          控制超级电容的充电功率和开关
 * @author         LYH
 * @param[in]      target_power：(15-200w)
                   flag:0：开;1：关
 * @retval         None
 */
void CAN_CMD_cap(uint16_t target_power, uint16_t flag) {
  uint32_t send_mail_box;
  capid_tx_message.StdId = CAN_CAPID;
  capid_tx_message.IDE = CAN_ID_STD;
  capid_tx_message.RTR = CAN_RTR_DATA;
  capid_tx_message.DLC = 0x08;
  capid_can_send_data[0] = target_power & 0xFF;
  capid_can_send_data[1] = target_power >> 8;
  capid_can_send_data[2] = flag & 0xFF;
  capid_can_send_data[3] = flag >> 8;
  HAL_CAN_AddTxMessage(&CAP_CAN, &capid_tx_message, capid_can_send_data, &send_mail_box);
}
/**
 * @brief          返回舵电机 6020电机数据指针
 * @author         LYH
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t* get_Forward_L_motor_measure_point(void) {
  return &Forward_L;
}
const motor_measure_t* get_Forward_R_motor_measure_point(void) {
  return &Forward_R;
}
const motor_measure_t* get_Back_R_motor_measure_point(void) {
  return &Back_R;
}
const motor_measure_t* get_Back_L_motor_measure_point(void) {
  return &Back_L;
}

/**
 * @brief          返回超级电容数据指针
 * @author         LYH
 * @param[in]      none
 * @retval         超级电容数据指针
 */
cap_measure_t* get_cap_measure_point(void) {
  return &get_cap;
}

/**
 * @brief          返回轮电机 3508电机数据指针
 * @author         LYH
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t* get_chassis_motor_measure_point(uint8_t i) {
  return &motor_chassis[(i & 0x03)];
}
