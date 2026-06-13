# SMT32下位机 快速熟悉

## 项目概览
- MCU: STM32F103C8T6, 系统时钟 72MHz (SystemInit)
- Keil 工程: PS2_Demo.uvprojx
- 主入口: USER/main.c
- 核心功能: PS2 手柄控制三电机闭环 + 三路 IMU 融合 + USART3 输出到 Jetson

## 快速开始
1. 用 Keil uVision 打开 PS2_Demo.uvprojx
2. 确认 USER/Hardware/System 目录的源文件已加入工程分组
3. 编译下载到板子
4. USART3 连接 Jetson/PC (PB10/PB11, 115200, 8N1)

> 注意: Keil 工程采用显式文件列表管理, 新增 Hardware 下的 .c/.h 需要手动加入工程分组, 否则不会参与编译。

## 硬件接线
| 模块 | 引脚 | 说明 |
| --- | --- | --- |
| PS2 接收器 | PB12/PB13/PB14/PB15 | DAT/CMD/CS/CLK |
| TB6612 PWM | PA8/PA9/PA10 | TIM1_CH1/CH2/CH3 |
| TB6612 方向 | PA2/PA3, PA4/PA5, PB0/PB1 | 三路方向控制 |
| 编码器1 | PA0/PA1 | TIM2 编码器模式 |
| 编码器2 | PA6/PA7 | TIM3 编码器模式 |
| 编码器3 | PB6/PB7 | TIM4 编码器模式 |
| IMU1 (软 I2C) | PB8/PB9 | SCL/SDA |
| IMU2 (软 I2C) | PA11/PA12 | SCL/SDA |
| IMU3 (软 I2C) | PB4/PB5 | SCL/SDA |
| Jetson 串口 | PB10/PB11 | USART3 TX/RX |

IMU 传感器组合默认: MPU6050 + HMC5883L + BMP180
- 7-bit 地址: MPU6050=0x68, HMC5883L=0x1E, BMP180=0x77
- 初始化会打开 MPU6050 BYPASS, 直连磁力计/气压计

## 软件结构 (关键文件)
- USER/main.c: 主循环, 摇杆映射, 三轮混控, 闭环更新, IMU 更新与串口输出
- USER/App.c: TaskRun, PS2 状态读取, SELECT+START 急停锁存, L1/R1 夹爪开合
- USER/PS2GamePad.c/h: PS2 通信, 按键/摇杆解析, 移动平均滤波(窗口 5)
- USER/Filter.c/h: 移动平均与一阶低通滤波工具
- Hardware/TIME.c/h: TIM1 PWM, TIM2/3/4 编码器, 闭环控制(Kp/Ki/Kff)
- Hardware/myMOTOR.c/h: TB6612 方向引脚控制
- Hardware/IMU_Multi.c/h: 三路软件 I2C, MPU/HMC/BMP 读取与校准
- Hardware/AHRS.c/h: Mahony 融合, 输出 roll/pitch/yaw (deg)
- Hardware/IMUSerial.c/h: USART3 打印 + 主机命令解析
- System/Delay.c/h: 基于 SysTick 的延时

## 控制与数据链路 (10ms 周期)
1. TaskRun 读取 PS2 手柄, 做中心夹紧与移动平均滤波
2. 主循环对摇杆做死区 + 指数曲线, 进入三轮分段解算
3. 速度目标 -> 目标计数(1/80) -> 一阶低通 -> 闭环控制输出 PWM
4. IMU_Multi_Update(0.01) 更新姿态, USART3 输出三路 IMU 数据

## 串口数据格式 (USART3, 115200)
启动时会发送表头:
- IMU1_HEADER,t,o,r,p,y,temp,press,alt
- IMU2_HEADER,t,o,r,p,y,temp,press,alt
- IMU3_HEADER,t,o,r,p,y,temp,press,alt

运行时每 10ms 输出 3 行:
- IMU1, t, online, roll, pitch, yaw, temp, press, alt
- IMU2, t, online, roll, pitch, yaw, temp, press, alt
- IMU3, t, online, roll, pitch, yaw, temp, press, alt

单位: t(s), roll/pitch/yaw(度), temp(C), press(Pa), alt(m)

## 关键参数与调参入口
- 控制周期: CTRL_DT_SEC = 0.01 (必须与 Delay_ms(10) 一致)
- 闭环增益: CTRL_KP=55, CTRL_KI=180, CTRL_KFF=12
- PWM 限幅: +/-7200 (与 TIM1 周期匹配)
- 摇杆: AXIS_DEADZONE_NORM=0.08, AXIS_EXPO=0.55

TIME.c 内含完整调参步骤: 先 Kp, 再 Ki, 最后 Kff, 每次修改后建议 ClosedLoop_Reset。

## 上位机工具
jetson_imu3_realtime_plot.py 支持串口实时曲线 + CSV 记录:
- 例: python jetson_imu3_realtime_plot.py --port COM5 --baud 115200 --window 500 --csv imu_pose_log.csv
- 支持两种协议: 分行的 IMU1/2/3, 以及单行 IMU3 聚合格式

## 上位机串口 / ROS 控制
当前下位机协议以 `docs/control_protocol.md` 为准。主机命令为 ASCII 行，行尾 `\n`：

- `J,<feed>,<x>,<y>`：虚拟摇杆速度命令，三个值范围均为 `-100..100`。
  - `feed`：进给 / 回撤。
  - `x`：左右弯曲。
  - `y`：上下弯曲。
- `S`：停止主机速度命令。
- `A,1` / `A,0`：使能 / 禁止机械臂电机输出。
- `E`：急停锁存。
- `G,<0..100>` 或 `G,O/C/S`：夹爪开度或开/合/停别名。
- `P,T/X/O/S`：三角、叉、圆、方形图案。
- `H`：软件回零/复位速度环。
- `Q`：请求一次 `ARM,enabled,estop,gripper,gripper_dir,home_pending` 状态帧。

主机速度命令超时约 200 ms；主机停止发送 `J` 帧后，下位机会回到中性/PS2 控制。当前 ROS 上位机链路为：

```text
arm_controller -> /arm/cmd (geometry_msgs/Twist)
serial_imu_bridge -> J,<feed>,<x>,<y> -> STM32 USART3
```

默认映射在 `RoboticArmControl_ROS/src/arm_serial_bridge/scripts/serial_imu_bridge.py` 中：

```text
/arm/cmd.linear.z -> feed
/arm/cmd.linear.x -> x
/arm/cmd.linear.y -> y
axis = clamp(round(value * cmd_scale * cmd_sign), -100, 100)
```

默认 `cmd_scale_* = 1000.0`，即 `0.1 m/s` 对应 `100`。方向和比例可通过 launch 参数 `cmd_sign_*`、`cmd_scale_*` 调整。

## 注意事项 / 易踩坑
- PB4 被 JTAG 占用, 已在 IMU_Multi_Init 内关闭 JTAG 仅保留 SWD
- PA11/PA12 与 USB/CAN 复用, 如启用相关功能需调整 IMU2 引脚
- 软件 I2C 为开漏输出, SCL/SDA 必须外接上拉
- 传感器型号变更(如 MPU9250/QMC5883/BMP280)需同步地址与寄存器
- 串口报文较长时需留意 IMUSerial_Printf 缓冲区大小(当前 256B)
