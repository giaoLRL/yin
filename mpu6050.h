/*
 * mpu6050.h — MPU6050 陀螺仪驱动
 *
 * I2C0: SCL=PA11, SDA=PA10
 * 量程: 陀螺 ±250°/s, 加速度计 ±2g
 * 角度: 互补滤波 (98% 陀螺 + 2% 加速度计)
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

/* 互补滤波时间常数 */
#define MPU6050_DT            0.005f    /* 5ms = 200Hz */
#define MPU6050_ALPHA         0.98f     /* 陀螺权重 */
#define MPU6050_GYRO_SENS     131.0f    /* LSB/(deg/s) @ ±250°/s */
#define MPU6050_ACCEL_SENS    16384.0f  /* LSB/g @ ±2g */

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 MPU6050: 唤醒, 设量程, 设 DLPF */
bool mpu6050_init(void);

/* 静态校准: 采集 N 次陀螺数据求零偏 (保持传感器静止) */
void mpu6050_calibrate(void);

/* 获取互补滤波后的欧拉角 (单位: 度) */
void mpu6050_get_angles(float *pitch, float *roll, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */