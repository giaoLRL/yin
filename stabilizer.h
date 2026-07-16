/*
 * stabilizer.h — 双轴云台增稳
 *
 * 两个独立 PID (偏航/俯仰) 控制两轴步进电机
 * 速率限制 + 死区 + 积分抗饱和
 */
#ifndef STABILIZER_H
#define STABILIZER_H

#include <stdbool.h>
#include <stdint.h>

/* PID 参数 (可在线调整) */
typedef struct {
    float kp, ki, kd;          /* 增益 */
    float integral_limit;       /* 积分限幅 */
    float output_limit;         /* 输出限幅 (Hz) */
    float dead_zone;            /* 死区 (度) */
    float rate_limit;           /* 速率限制 (Hz/周期) */
} PIDTuning;

/* PID 运行时状态 */
typedef struct {
    PIDTuning tune;
    float setpoint;             /* 目标角度 (度) */
    float integral;
    float prev_measurement;
    float prev_error;
    float prev_output;
} PIDState;

#ifdef __cplusplus
extern "C" {
#endif

/* PID 初始化 */
void pid_init(PIDState *pid, const PIDTuning *tune);

/* PID 一次迭代, 返回输出 (可正可负) */
float pid_compute(PIDState *pid, float measurement, float dt);

/* 增稳器: 封装两个 PID + 电机控制 */
typedef struct {
    PIDState yaw_pid;
    PIDState pitch_pid;
    bool     enabled;
} Stabilizer;

/* 增稳初始化 */
void stabilizer_init(Stabilizer *stab);

/* 增稳循环迭代 (200Hz: dt=0.005s) */
void stabilizer_update(Stabilizer *stab);

/* 启用/停用增稳 */
void stabilizer_set_enabled(Stabilizer *stab, bool en);

#ifdef __cplusplus
}
#endif

#endif /* STABILIZER_H */