# Rudder Swerve Control

面向 RoboMaster 步兵底盘的四舵轮控制核心。该仓库只公开底盘运动学、控制状态机、CAN 电机接口及其必要算法，不包含整车业务、上位机、UI、裁判系统或完整板级工程。

> **安全提示**：首次上电请将机器人架空，确认四个舵向零位、轮组编号、CAN ID 与电机方向后再使能输出。

## 功能

- 四舵轮速度/方位角解算；
- 舵向最短路径、最优角与 `cos^5` 轮速补偿；
- 跟随云台、舵轮跟随、旋转及失能状态机；
- 舵向位置环、轮速环与功率控制接口；
- CAN 电机反馈与离线状态接口。

## 代码范围

| 路径 | 内容 |
| --- | --- |
| `firmware/User/Application` | 底盘任务、行为状态机、CAN 收发接口 |
| `firmware/User/algorithm` | PID、斜坡与通用数学工具 |
| `firmware/User/modules` | 电机数据结构与功率控制接口 |

这是从实车工程中抽取的控制层，不是开箱即烧录的完整固件。移植时须由使用者提供 STM32 HAL、FreeRTOS、IMU/云台反馈、裁判系统数据和电机 CAN 驱动；详见 [移植说明](docs/porting.md)。

## 控制链路

```mermaid
flowchart LR
  A[遥控器 / 云台指令] --> B[行为状态机]
  B --> C[四舵轮解算]
  C --> D[最短转角与轮速反转]
  D --> E[舵向位置环 / 轮速环]
  E --> F[CAN 电机输出]
  G[编码器与电机反馈] --> E
```

## 舵向优化

控制实现位于 [`chassis_task.c`](firmware/User/Application/src/chassis_task.c) 的 `Rudder_motor_relative_angle_control`：

1. 将舵向编码器误差折返到 `[-4096, 4096]`，即选择不超过 180° 的转动路径。
2. 当绝对误差超过 `2048`（90°）时，将目标等效旋转 180°，并把对应驱动轮速度置反；舵机最终只需转动不超过 90°，这是舵轮的最优角/就近原则。
3. 轮速乘以 `cos(error)^5`。舵轮尚未对准时，驱动轮速度被平滑压低；误差趋近 90° 时趋近零，以减小侧向拖拽。这里是五次余弦补偿，不是一次 `cos` 补偿。

编码器一圈按 8192 计数；移植到其他绝对编码器时，应同步修改零位、满量程和 90° 阈值。

## 硬件与环境

- 参考 MCU：STM32F407；
- 参考系统：FreeRTOS、STM32 HAL、CMSIS-DSP；
- 参考执行机构：4 个驱动电机 + 4 个舵向电机，经 CAN 通信；
- 轮径、轮距、零位编码器值和 CAN ID 均需按实物配置。

## 快速移植

1. 将 `firmware/User` 合并到自己的 STM32 工程并配置 include path。
2. 实现或适配 `CAN_cmd_rudder`、`CAN_cmd_chassis` 及电机反馈接口。
3. 在 `chassis_task.h` 配置四个舵机的编码器零位和 PID 初值。
4. 在 `chassis_task.c` 校准轮组半径、轮到中心距离、方位角和方向。
5. 架空底盘，依次验证单轮舵向、单轮驱动、平移、原地旋转与失联保护。

## 演示与参考资料

- [视频 1](https://www.bilibili.com/video/BV13p4y177HZ/)
- [视频 2](https://www.bilibili.com/video/BV1YET3zsEG1/)
- [视频 3](https://www.bilibili.com/video/BV1HXT4z7EcB/)
- [视频 4](https://www.bilibili.com/video/BV11utSzqE4z/)
- [视频 5](https://www.bilibili.com/video/BV1BF7NzzELk/)
- [视频 6](https://www.bilibili.com/video/BV1o5HfzYEaa/)

以上链接仅作演示或参考资料；发布前请标注每个视频的作者、拍摄日期、对应固件版本和测试项目。

## 许可证与致谢

本仓库中由发布者拥有著作权的原创代码采用 [MIT License](LICENSE)。如引入第三方 SDK、库或历史代码，使用前须确认其许可证与署名要求；它们不因本许可证而被重新授权。
