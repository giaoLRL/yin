# 破云台2 — 双轴陀螺增稳云台

基于 MSPM0G3507 的双轴云台增稳系统，使用 MPU6050 陀螺仪 + 互补滤波 + PID 控制 + 步进电机驱动。

## 硬件
- **MCU**: MSPM0G3507 (Cortex-M0+, 32MHz)
- **传感器**: MPU6050 (I2C0, PA10=SDA, PA11=SCL)
- **电机A (偏航)**: ST=PA15(PWM_1), DIR=PB13, EN=PB15
- **电机B (俯仰)**: ST=PB14(PWM_0), DIR=PB16, EN=PA12

## 软件架构
1. MPU6050 互补滤波解算姿态角度 (200Hz)
2. 双轴独立 PID 控制器 (带死区、抗饱和、速率限制)
3. PWM 脉冲驱动步进电机
4. SysConfig 生成硬件初始化代码

## 构建
使用 TI Code Composer Studio Theia + MSPM0 SDK 2.10.00.04