#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include "stm32f1xx_hal.h"
#include "algorithm_pid.h"
#include "bsp_motor.h"

// --- 配置参数 ---
#define ROBOT_DT           0.02f  // 控制周期 10ms
#define SAFETY_TIMEOUT_MS  5000    // 超时停机
#define MOTOR_DEADZONE  3000.0f
#define MOTOR_DEADZONE_lite 1.0f

#define KP 20.0f
#define KI 1.0f
#define KD 0.0f
// --- 速度设定 (单位: RPM) ---
// 物理限制: Max = 130 RPM
// 设定原则:
// 1. Normal: 约 40% 满速，保证控制细腻。
// 2. Turbo:  约 90% 满速。注意：如果是麦轮斜跑 (Vx+Vy)，两个速度叠加可能会超过130，
//            但这没关系，PID的 output_limit 会自动截断。
#define SPEED_RPM_NORMAL   25.0f
#define SPEED_RPM_TURBO    100.0f
#define SPEED_RPM_ROT_NORMAL  30.0f  // 正常旋转速度

// --- 机器人对象 ---
typedef struct {
    // 1. 输入状态
    struct {
        uint8_t KeyMask;
        uint32_t LastPacketTick;
    } Remote;

    // 2. 期望速度 (Target RPM)
    float Target_Vx;
    float Target_Vy;

    // 3. PID 控制器
    PID_t Pid_LF;
    PID_t Pid_RF;
    PID_t Pid_LB;
    PID_t Pid_RB;

} Robot_TypeDef;

// --- API ---
void Robot_Init(void);
void Robot_UART_IDLE_Handler(UART_HandleTypeDef *huart);
void Robot_Periodic_Task(void);
void Robot_Protocol_Process(void);
#endif // APP_ROBOT_H