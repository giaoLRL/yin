/*
 * step_motor.c — 双轴步进电机驱动（完全使用 SysConfig）
 *
 * 引脚（来自 SysConfig）:
 *   电机A: ST=PA15(PWM_1/TIMA1), DIR=PB13, EN=PB15
 *   电机B: ST=PB14(PWM_0/TIMA0), DIR=PB16, EN=PA12
 *
 * 不手动初始化任何硬件，SYSCFG_DL_init() 已完成一切。
 * 本文件只做: DIR 默认值 + 频率更新 + 启停。
 */
#include "step_motor.h"

#define PWM_DEFAULT_FREQ  1000
#define PWM_DUTY_PERCENT  50
#define PWM_MIN_FREQ      20
#define PWM_MAX_FREQ      5000

void step_motor_init(void)
{
    /* DIR 默认 HIGH (CW). EN 保持 SysConfig 的 LOW */
    DL_GPIO_setPins(GPIOB, STEP_MOTOR_A_DIR1_PIN);  /* PB13 */
    DL_GPIO_setPins(GPIOB, STEP_MOTOR_B_DIR2_PIN);  /* PB16 */
}

void step_motor_enable(StepMotorID motor, uint8_t enable)
{
    if (motor == STEP_MOTOR_A) {
        if (enable) DL_GPIO_setPins(GPIOB, STEP_MOTOR_A_EN1_PIN);
        else        DL_GPIO_clearPins(GPIOB, STEP_MOTOR_A_EN1_PIN);
    } else {
        if (enable) DL_GPIO_setPins(GPIOA, STEP_MOTOR_B_EN2_PIN);
        else        DL_GPIO_clearPins(GPIOA, STEP_MOTOR_B_EN2_PIN);
    }
}

void step_motor_set_dir(StepMotorID motor, StepMotorDir dir)
{
    if (motor == STEP_MOTOR_A) {
        if (dir == STEP_DIR_CW) DL_GPIO_setPins(GPIOB, STEP_MOTOR_A_DIR1_PIN);
        else                    DL_GPIO_clearPins(GPIOB, STEP_MOTOR_A_DIR1_PIN);
    } else {
        if (dir == STEP_DIR_CW) DL_GPIO_setPins(GPIOB, STEP_MOTOR_B_DIR2_PIN);
        else                    DL_GPIO_clearPins(GPIOB, STEP_MOTOR_B_DIR2_PIN);
    }
}

void step_motor_start(StepMotorID motor, uint32_t freq_hz)
{
    if (freq_hz == 0) freq_hz = PWM_DEFAULT_FREQ;
    /* 限制频率范围，防止 period 下溢 */
    if (freq_hz < PWM_MIN_FREQ) freq_hz = PWM_MIN_FREQ;
    if (freq_hz > PWM_MAX_FREQ) freq_hz = PWM_MAX_FREQ;

    uint32_t period  = CPUCLK_FREQ / freq_hz;
    uint32_t compare = (period * PWM_DUTY_PERCENT) / 100;

    if (motor == STEP_MOTOR_A) {
        DL_Timer_stopCounter(PWM_1_INST);
        DL_Timer_setLoadValue(PWM_1_INST, period - 1);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, compare,
            (DL_TIMER_CC_INDEX)DL_TIMER_CC_0_INDEX);
        DL_Timer_startCounter(PWM_1_INST);
    } else {
        DL_Timer_stopCounter(PWM_0_INST);
        DL_Timer_setLoadValue(PWM_0_INST, period - 1);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, compare,
            (DL_TIMER_CC_INDEX)DL_TIMER_CC_0_INDEX);
        DL_Timer_startCounter(PWM_0_INST);
    }
}

void step_motor_stop(StepMotorID motor)
{
    if (motor == STEP_MOTOR_A)
        DL_Timer_stopCounter(PWM_1_INST);
    else
        DL_Timer_stopCounter(PWM_0_INST);
}

void step_motor_update_freq(StepMotorID motor, uint32_t freq_hz)
{
    /* 频率 0 则停止 */
    if (freq_hz == 0) {
        step_motor_stop(motor);
        return;
    }

    /* 限制频率范围 */
    if (freq_hz < PWM_MIN_FREQ) freq_hz = PWM_MIN_FREQ;
    if (freq_hz > PWM_MAX_FREQ) freq_hz = PWM_MAX_FREQ;

    uint32_t period  = CPUCLK_FREQ / freq_hz;
    uint32_t compare = (period * PWM_DUTY_PERCENT) / 100;

    if (motor == STEP_MOTOR_A) {
        DL_Timer_setLoadValue(PWM_1_INST, period - 1);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, compare,
            (DL_TIMER_CC_INDEX)DL_TIMER_CC_0_INDEX);
        DL_Timer_startCounter(PWM_1_INST);
    } else {
        DL_Timer_setLoadValue(PWM_0_INST, period - 1);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, compare,
            (DL_TIMER_CC_INDEX)DL_TIMER_CC_0_INDEX);
        DL_Timer_startCounter(PWM_0_INST);
    }
}

void step_motor_move(StepMotorID motor, StepMotorDir dir,
                     uint32_t steps, uint32_t freq_hz)
{
    if (freq_hz == 0) freq_hz = PWM_DEFAULT_FREQ;

    step_motor_set_dir(motor, dir);
    step_motor_start(motor, freq_hz);

    uint32_t total_us = (uint32_t)((uint64_t)steps * 1000000 / freq_hz);
    delay_us(total_us);

    step_motor_stop(motor);
}

void delay_us(uint32_t us)
{
    delay_cycles((uint64_t)us * (CPUCLK_FREQ / 1000000));
}