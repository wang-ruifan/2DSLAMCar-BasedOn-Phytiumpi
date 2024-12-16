# 基于飞腾派多核异构的2d激光SLAM小车

# For English version, please click [here](#2d-slam-car-basedon-phytiumpi)

## 项目背景
这是我参加2024年第八届全国大学生集成电路创新大赛-飞腾杯的项目，题目为基于飞腾多核异构处理器的创新应用开发。  
项目基于飞腾派，利用飞腾派多核异构处理器，主核运行Ubuntu，从核运行FreeRTOS，同时实现小车底盘控制和SLAM建图。

## 项目功能
- 基于飞腾派多核异构处理器，主核运行Ubuntu，从核运行FreeRTOS
- 从核实现底盘控制，包含后轮电机转速闭环控制，前轮舵机转向控制，并通过红外传感器检测前方障碍物实现紧急制动
- 主核提供人机交互界面对小车进行控制，同时通过激光雷达和里程计数据融合建图

### 详细实现原理
基于OpenAMP协议，首先四个核启动Ubuntu，然后切换为三个核运行Ubuntu(主核)，一个核运行FreeRTOS(从核)

#### 从核：FreeRTOS
1. rpmsg-echo_os.c
- 运行RpmsgEchoTask，注册RPMsg信息接收回调函数，在其中接收目标速度Vx及Vz
- 对目标速度进行运动学求解，转向半径R=Vx/Vz，转向角度AngleR=atan(轮距/R)
- 根据舵机角度与脉冲转换公式，得到舵机脉冲与后轮目标速度
2. car.c
- 运行CarInitTask，初始化GPIO，通过IOPad设置引脚方向，注册中断回调函数
- 编码器A输出上升沿时触发中断，根据B的电平对当前周期的电机编码器脉冲苏加/减
- 循环运行CarControlTask，根据连接红外传感器的引脚的电平，判断前方是否有障碍物，若有则期望速度调为0
- 根据编码器脉冲数，得到电机实际速度
- 调用PI控制器，根据实际速度和目标速度得到PWM输出脉冲数
- 控制PWM输出特定脉冲数

#### 主核：Ubuntu 20.04
1. mainwindow.cpp
- 通过基于OpenAMP协议中的RPMsg通信，向从核发送目标速度并接收从核发来的实时速度Vx及Vz
- 运行基于Qt的上位机，通过人机交互界面的摇杆输入目标速度，并显示实时速度
2. car_gui.cpp
- 基于实时速度数据在/odom话题中发布里程计tf数据

- ROS中运行Cartographer，基于2d激光雷达数据和里程计进行实时建图

## 文件目录
```
2D-SLAM-car-basedon-phytiumpi/
├── master-src/         # 主核ROS包
├── slave-src/          # 从核源代码
├── docs/               # 文档
├── executable-files/   # 可执行文件
└── README.md           # 项目介绍
```

## 环境配置


## 编译


## 使用


更多详细信息请参考项目文档。

# 2D-SLAM-car-basedon-phytiumpi