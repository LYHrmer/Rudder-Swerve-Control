# 配置参数

| 参数 | 文件 | 含义 | 修改时机 |
| --- | --- | --- | --- |
| `Forward_L_ecd` 等 | `chassis_task.h` | 四个舵向零位编码器值 | 机械装配或编码器更换后 |
| `CHASSIS_KP/KI/KD` | `chassis_task.h` | 驱动轮速度环 PID | 驱动轮速度测试后 |
| `RUDDER_*` | `chassis_task.h` | 舵向位置/速度环参数 | 架空单轮调试后 |
| `MAX_WHEEL_SPEED` | `chassis_task.h` | 轮速上限 | 按电机和比赛策略设定 |
| `Wheel_Radius` | `chassis_task.c` | 轮半径 | 轮组规格变化后 |
| `Wheel_To_Core_Distance` | `chassis_task.c` | 轮组到旋转中心距离 | 底盘几何变更后 |
| `Wheel_Azimuth` | `chassis_task.c` | 轮组方位角 | 轮组布局变化后 |

参数必须结合实车测得数据填写。不要直接把本仓库的 PID 与编码器零位用于另一台机器人。
