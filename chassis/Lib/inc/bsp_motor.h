#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>
#include "protocol.h"
/*
 * bsp_motor.h
 *
 * Abstraction Layer for Motor Hardware.
 * Responsibilities:
 * 1. Initialize PWM and Encoder timers.
 * 2. Map logical motor channels (FL, FR...) to hardware timers.
 * 3. Convert raw hardware counters to RPM.
 */

/* Motor channels mapping */
typedef enum {
    MOTOR_CH_FL = 0, // Front Left
    MOTOR_CH_FR = 1, // Front Right
    MOTOR_CH_RL = 2, // Rear Left
    MOTOR_CH_RR = 3, // Rear Right
    MOTOR_CH_COUNT   // Total count for iteration
} e_motor_ch;

/*
 * Initialization
 * call this once before the main loop.
 */
void bsp_motor_init(void);

/*
 * Actuation: Set Output Power
 * @param ch: Motor channel
 * @param pwm: Control value range [-MAX_PWM .. +MAX_PWM]
 *             Positive = Forward, Negative = Reverse
 */
void bsp_motor_set_pwm(e_motor_ch ch, int32_t pwm);

/*
 * Sensing: Get Speed
 * @param ch: Motor channel
 * @return: Speed in RPM (Rotations Per Minute)
 */
float bsp_motor_get_rpm(e_motor_ch ch);

/*
 * Periodic Update Task
 * Must be called at a fixed frequency (e.g., 100Hz or 1kHz)
 * to process encoder counters and calculate speed.
 * @param dt_s: Delta time in seconds (e.g., 0.01f for 10ms)
 */
void bsp_motor_periodic_update(float dt_s);

#endif /* BSP_MOTOR_H */