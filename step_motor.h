#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STEP_MOTOR_A = 0,   /* 电机A: ST=PA15(PWM_1), DIR=PB13, EN=PB15 */
    STEP_MOTOR_B = 1    /* 电机B: ST=PB14(PWM_0), DIR=PB16, EN=PA12 */
} StepMotorID;

typedef enum {
    STEP_DIR_CW  = 1,
    STEP_DIR_CCW = 0
} StepMotorDir;

void step_motor_init(void);
void step_motor_enable(StepMotorID motor, uint8_t enable);
void step_motor_set_dir(StepMotorID motor, StepMotorDir dir);
void step_motor_start(StepMotorID motor, uint32_t freq_hz);
void step_motor_stop(StepMotorID motor);
void step_motor_update_freq(StepMotorID motor, uint32_t freq_hz);
void step_motor_move(StepMotorID motor, StepMotorDir dir,
                     uint32_t steps, uint32_t freq_hz);
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif