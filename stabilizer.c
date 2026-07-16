/*
 * stabilizer.c — 双轴云台增稳实现
 *
 * PID + 速率限制 + 死区 + 电机独立控制
 * 200Hz 循环: 读 MPU6050 → 互补滤波 → PID → 更新电机频率
 */
#include "ti_msp_dl_config.h"
#include "step_motor.h"
#include "mpu6050.h"
#include "stabilizer.h"
#include <math.h>
#include <stddef.h>

/* ---- PID ---- */

void pid_init(PIDState *pid, const PIDTuning *tune)
{
    pid->tune             = *tune;
    pid->setpoint         = 0.0f;
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_error       = 0.0f;
    pid->prev_output      = 0.0f;
}

float pid_compute(PIDState *pid, float measurement, float dt)
{
    const PIDTuning *t = &pid->tune;

    /* 1. 误差 = 目标 - 测量 */
    float error = pid->setpoint - measurement;

    /* 2. 死区 */
    bool in_deadzone = (fabsf(error) < t->dead_zone);
    if (in_deadzone) {
        error = 0.0f;
    }

    /* 3. 比例项 */
    float p_term = t->kp * error;

    /* 4. 积分项 (梯形积分 + 抗饱和); 死区内不累积，防止 prev_error 残留导致过充 */
    if (!in_deadzone) {
        pid->integral += 0.5f * (error + pid->prev_error) * dt;
        if (pid->integral >  t->integral_limit) pid->integral =  t->integral_limit;
        if (pid->integral < -t->integral_limit) pid->integral = -t->integral_limit;
    }
    float i_term = t->ki * pid->integral;

    /* 5. 微分项 (测量微分, 防设定值冲击) */
    float d_term = -t->kd * (measurement - pid->prev_measurement) / dt;

    /* 6. 合成输出 */
    float output = p_term + i_term + d_term;

    /* 7. 输出钳位 */
    if (output >  t->output_limit) output =  t->output_limit;
    if (output < -t->output_limit) output = -t->output_limit;

    /* 8. 速率限制 */
    float delta = output - pid->prev_output;
    if (delta >  t->rate_limit) output = pid->prev_output + t->rate_limit;
    if (delta < -t->rate_limit) output = pid->prev_output - t->rate_limit;

    /* 9. 保存状态 */
    pid->prev_measurement = measurement;
    pid->prev_error       = error;
    pid->prev_output      = output;

    return output;
}

/* ---- 默认 PID 参数 ---- */

static const PIDTuning g_default_yaw_tune = {
    .kp             = 0.5f,
    .ki             = 0.02f,
    .kd             = 0.1f,
    .integral_limit = 200.0f,
    .output_limit   = 800.0f,
    .dead_zone      = 0.3f,
    .rate_limit     = 100.0f,
};

static const PIDTuning g_default_pitch_tune = {
    .kp             = 0.5f,
    .ki             = 0.02f,
    .kd             = 0.1f,
    .integral_limit = 100.0f,
    .output_limit   = 600.0f,
    .dead_zone      = 0.3f,
    .rate_limit     = 100.0f,
};

/* ---- 增稳器 ---- */

void stabilizer_init(Stabilizer *stab)
{
    pid_init(&stab->yaw_pid,   &g_default_yaw_tune);
    pid_init(&stab->pitch_pid, &g_default_pitch_tune);
    stab->enabled = false;
}

void stabilizer_set_enabled(Stabilizer *stab, bool en)
{
    stab->enabled = en;
}

void stabilizer_update(Stabilizer *stab)
{
    if (!stab->enabled) return;

    /* 1. 读取角度 */
    float pitch, roll, yaw;
    mpu6050_get_angles(&pitch, &roll, &yaw);

    /* 2. PID 计算 (pitch→俯仰电机B, yaw→偏航电机A) */
    float dt = MPU6050_DT;

    float yaw_correction   = pid_compute(&stab->yaw_pid,   yaw,   dt);
    float pitch_correction = pid_compute(&stab->pitch_pid, pitch, dt);

    /* 3. 电机 A (偏航) */
    {
        float    freq = fabsf(yaw_correction);
        StepMotorDir dir = (yaw_correction >= 0.0f) ? STEP_DIR_CW : STEP_DIR_CCW;

        step_motor_set_dir(STEP_MOTOR_A, dir);
        if (freq < 1.0f) {
            step_motor_stop(STEP_MOTOR_A);
        } else {
            step_motor_update_freq(STEP_MOTOR_A, (uint32_t)freq);
        }
    }

    /* 4. 电机 B (俯仰) */
    {
        float    freq = fabsf(pitch_correction);
        StepMotorDir dir = (pitch_correction >= 0.0f) ? STEP_DIR_CW : STEP_DIR_CCW;

        step_motor_set_dir(STEP_MOTOR_B, dir);
        if (freq < 1.0f) {
            step_motor_stop(STEP_MOTOR_B);
        } else {
            step_motor_update_freq(STEP_MOTOR_B, (uint32_t)freq);
        }
    }
}