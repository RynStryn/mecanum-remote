#ifndef PROTOCOL_ARM_H
#define PROTOCOL_ARM_H

#include <stdint.h>

/* --- Packet Constants --- */
#define PKT_HEAD        0xFF
#define PKT_TAIL        0xFE
#define PKT_TYPE_ARM    0x02
#define PKT_LEN         10U  // Fixed Length 10 Bytes

/* PWM Parameters for 50Hz Servo (基于 72MHz 时钟) */
#define SERVO_PWM_ARR  	19999 // (36MHz / (1+71)) * 20000 = 50Hz 
                              // (注: TIM2/3 在 APB1, Max 36MHz, 见 TIM.c 注释)
#define SERVO_MIN_PULSE 500   // ~0.5ms (Min angle)
#define SERVO_MAX_PULSE 2500  // ~2.5ms (Max angle)
#define SERVO_RANGE_OUT (SERVO_MAX_PULSE - SERVO_MIN_PULSE) // 2000

#endif
