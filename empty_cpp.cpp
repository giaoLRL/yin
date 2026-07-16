#include "ti_msp_dl_config.h"
#include "step_motor.h"
#include "mpu6050.h"
#include "stabilizer.h"

/*
 * 双轴云台 — 陀螺增稳
 *
 * 引脚 (SysConfig):
 *   电机A (偏航): ST=PA15(PWM_1/TIMA1), DIR=PB13, EN=PB15
 *   电机B (俯仰): ST=PB14(PWM_0/TIMA0), DIR=PB16, EN=PA12
 *   MPU6050:      SCL=PA11(I2C0), SDA=PA10(I2C0)
 *
 * 流程:
 *   1. 初始化 I2C + MPU6050 + PID
 *   2. 电机测试 (验证硬件正常)
 *   3. 进入增稳模式 (200Hz)
 */

int main(void)
{
    SYSCFG_DL_init();
    step_motor_init();

    /* ===== MPU6050 初始化 ===== */
    if (!mpu6050_init()) {
        /* MPU6050 初始化失败, 闪烁电机提示 (简单来回转) */
        while (1) {
            step_motor_enable(STEP_MOTOR_A, 1);
            step_motor_move(STEP_MOTOR_A, STEP_DIR_CW,  10, 200);
            step_motor_move(STEP_MOTOR_A, STEP_DIR_CCW, 10, 200);
            step_motor_enable(STEP_MOTOR_A, 0);
            delay_us(500000);
        }
    }

    /* 陀螺零偏校准 (保持传感器静止!) */
    mpu6050_calibrate();

    /* ===== 增稳器初始化 ===== */
    Stabilizer stab;
    stabilizer_init(&stab);

    /* ===== 使能电机 ===== */
    step_motor_enable(STEP_MOTOR_A, 1);
    step_motor_enable(STEP_MOTOR_B, 1);

    /* ===== 电机测试: 验证硬件正常 ===== */
    step_motor_move(STEP_MOTOR_A, STEP_DIR_CW,  200, 800);
    step_motor_move(STEP_MOTOR_A, STEP_DIR_CCW, 200, 800);
    step_motor_move(STEP_MOTOR_B, STEP_DIR_CW,  200, 800);
    step_motor_move(STEP_MOTOR_B, STEP_DIR_CCW, 200, 800);

    /* ===== 进入增稳 ===== */
    stabilizer_set_enabled(&stab, true);

    /* ===== 200Hz 增稳主循环 ===== */
    while (1) {
        stabilizer_update(&stab);
        delay_us(4000);  /* ~4ms → 总周期 ≈ 5ms (200Hz) */
    }
}