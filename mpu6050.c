/*
 * mpu6050.c — MPU6050 陀螺仪驱动实现
 *
 * 使用 SysConfig 生成的 I2C0 (PA10=SDA, PA11=SCL)
 * 互补滤波融合陀螺和加速度计数据
 */
#include "ti_msp_dl_config.h"
#include "mpu6050.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 寄存器地址 */
#define MPU6050_ADDR            0x68
#define REG_WHO_AM_I            0x75
#define REG_PWR_MGMT1           0x6B
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_XOUT_H        0x3B
#define REG_GYRO_XOUT_H         0x43

/* 校准采样数 */
#define CALIB_SAMPLES           200

/* I2C 超时 (循环次数) */
#define I2C_TIMEOUT             100000

/* ---- 状态变量 ---- */
static float g_pitch_angle;     /* 俯仰角 (度) */
static float g_roll_angle;      /* 横滚角 (度) */
static float g_yaw_angle;       /* 偏航角 (度) */

static float g_gyro_bias_x;     /* 陀螺零偏 X */
static float g_gyro_bias_y;     /* 陀螺零偏 Y */
static float g_gyro_bias_z;     /* 陀螺零偏 Z */

/* ---- I2C 底层 ---- */

/* 写单个寄存器: start + dev(W) + reg + data + stop */
static bool mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    /* 等待传输完成 */
    uint32_t timeout = I2C_TIMEOUT;
    uint32_t status;
    do {
        status = DL_I2C_getControllerStatus(I2C_0_INST);
        if (--timeout == 0) return false;
    } while ((status & DL_I2C_CONTROLLER_STATUS_IDLE) == 0);

    if (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                  DL_I2C_CONTROLLER_STATUS_ADDR_ACK  |
                  DL_I2C_CONTROLLER_STATUS_DATA_ACK)) {
        return false;
    }
    return true;
}

/* 批量读寄存器: start + dev(W) + reg + restart + dev(R) + data[n] + stop */
static bool mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint32_t status;
    uint32_t timeout;

    /* Phase 1: 发送寄存器地址 (START, NO_STOP) */
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_ENABLE);

    timeout = I2C_TIMEOUT;
    do {
        status = DL_I2C_getControllerStatus(I2C_0_INST);
        if (--timeout == 0) return false;
    } while ((status & DL_I2C_CONTROLLER_STATUS_IDLE) == 0);

    /* Phase 2: 读取数据 (START, STOP) */
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_startControllerTransfer(I2C_0_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    timeout = I2C_TIMEOUT;
    do {
        status = DL_I2C_getControllerStatus(I2C_0_INST);
        if (--timeout == 0) return false;
    } while ((status & DL_I2C_CONTROLLER_STATUS_IDLE) == 0);

    /* 检查错误标志 (ADDR_ACK/DATA_ACK 位置1表示 NACK) */
    if (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                  DL_I2C_CONTROLLER_STATUS_ADDR_ACK  |
                  DL_I2C_CONTROLLER_STATUS_DATA_ACK)) {
        return false;
    }

    /* 读 RX FIFO */
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }

    return true;
}

/* ---- 初始化 ---- */

bool mpu6050_init(void)
{
    /* 1. 验证 WHO_AM_I */
    uint8_t whoami = 0;
    if (!mpu6050_read_regs(REG_WHO_AM_I, &whoami, 1)) return false;
    if (whoami != 0x68) return false;

    /* 2. 唤醒 (清除 SLEEP 位) */
    if (!mpu6050_write_reg(REG_PWR_MGMT1, 0x00)) return false;

    /* 等待稳定 */
    for (volatile int i = 0; i < 320000; i++) { __asm("nop"); }  /* ~10ms */

    /* 3. 采样率分频: 0 → 1kHz (陀螺), DLPF 开启后自动匹配 */
    if (!mpu6050_write_reg(REG_SMPLRT_DIV, 0x00)) return false;

    /* 4. DLPF: BW=44Hz (accel+gyro), 1kHz internal sample rate */
    if (!mpu6050_write_reg(REG_CONFIG, 0x03)) return false;

    /* 5. 陀螺量程: ±250°/s */
    if (!mpu6050_write_reg(REG_GYRO_CONFIG, 0x00)) return false;

    /* 6. 加速度计量程: ±2g */
    if (!mpu6050_write_reg(REG_ACCEL_CONFIG, 0x00)) return false;

    return true;
}

/* ---- 校准 ---- */

void mpu6050_calibrate(void)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        uint8_t buf[14];
        if (!mpu6050_read_regs(REG_ACCEL_XOUT_H, buf, 14)) continue;
        int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
        int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
        int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
        sum_gx += (float)gx;
        sum_gy += (float)gy;
        sum_gz += (float)gz;
        /* 简单延迟 ~1ms */
        for (volatile int j = 0; j < 32000; j++) { __asm("nop"); }
    }

    g_gyro_bias_x = sum_gx / (float)CALIB_SAMPLES;
    g_gyro_bias_y = sum_gy / (float)CALIB_SAMPLES;
    g_gyro_bias_z = sum_gz / (float)CALIB_SAMPLES;

    /* 清零角度初始值 */
    g_pitch_angle = 0.0f;
    g_roll_angle  = 0.0f;
    g_yaw_angle   = 0.0f;
}

/* ---- 互补滤波角度 ---- */

void mpu6050_get_angles(float *pitch, float *roll, float *yaw)
{
    uint8_t buf[14];
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;

    if (!mpu6050_read_regs(REG_ACCEL_XOUT_H, buf, 14)) {
        /* 读失败, 返回上一次的有效值 */
        *pitch = g_pitch_angle;
        *roll  = g_roll_angle;
        *yaw   = g_yaw_angle;
        return;
    }

    /* 大端→小端 */
    ax_raw = (int16_t)((buf[0]  << 8) | buf[1]);
    ay_raw = (int16_t)((buf[2]  << 8) | buf[3]);
    az_raw = (int16_t)((buf[4]  << 8) | buf[5]);
    gx_raw = (int16_t)((buf[8]  << 8) | buf[9]);
    gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
    gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

    /* 转物理单位 */
    float accel_x = (float)ax_raw / MPU6050_ACCEL_SENS;
    float accel_y = (float)ay_raw / MPU6050_ACCEL_SENS;
    float accel_z = (float)az_raw / MPU6050_ACCEL_SENS;
    float gyro_x  = ((float)gx_raw - g_gyro_bias_x) / MPU6050_GYRO_SENS;
    float gyro_y  = ((float)gy_raw - g_gyro_bias_y) / MPU6050_GYRO_SENS;
    float gyro_z  = ((float)gz_raw - g_gyro_bias_z) / MPU6050_GYRO_SENS;

    /* 加速度计角度 (度) */
    float accel_pitch = atan2f(-accel_x,
        sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;
    float accel_roll  = atan2f(accel_y, accel_z) * 180.0f / M_PI;

    /* 互补滤波 */
    const float alpha = MPU6050_ALPHA;
    const float dt    = MPU6050_DT;

    /* 轴映射: pitch(绕Y轴) 用 gyro_y, roll(绕X轴) 用 gyro_x */
    g_pitch_angle = alpha * (g_pitch_angle + gyro_y * dt)
                    + (1.0f - alpha) * accel_pitch;
    g_roll_angle  = alpha * (g_roll_angle  + gyro_x * dt)
                    + (1.0f - alpha) * accel_roll;
    g_yaw_angle  += gyro_z * dt;  /* 偏航只有陀螺积分 */

    *pitch = g_pitch_angle;
    *roll  = g_roll_angle;
    *yaw   = g_yaw_angle;
}