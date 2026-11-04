/* --- START OF FILE App_Robot.c (No RingBuffer Version) --- */

#include "../inc/App_Robot.h"
#include <string.h>

/* 全局变量 */
Robot_TypeDef Robot;

/* 外部句柄 */
extern UART_HandleTypeDef huart2; // 遥控接收
extern UART_HandleTypeDef huart3; // 机械臂转发
extern TIM_HandleTypeDef htim8;   // PWM

/* --- 1. 接收缓冲区 --- */
// 只需要容纳一个包的长度 (10字节)，给稍微大一点防止溢出即可
#define RX_BUF_SIZE 20
static uint8_t RxBuffer[RX_BUF_SIZE];

/* --- 初始化 --- */
void Robot_Init(void) {
    memset(&Robot, 0, sizeof(Robot_TypeDef));
    bsp_motor_init();

    // PID 参数初始化 (略)
    float kp =KP, ki = KI, kd = KD;
    PID_Init(&Robot.Pid_LF, kp, ki, kd, ROBOT_DT, 3500, 7199);
    PID_Init(&Robot.Pid_RF, kp, ki, kd, ROBOT_DT, 3500, 7199);
    PID_Init(&Robot.Pid_LB, kp, ki, kd, ROBOT_DT, 3500, 7199);
    PID_Init(&Robot.Pid_RB, kp, ki, kd, ROBOT_DT, 3500, 7199);

    // 启动 DMA 接收 (指向 RxBuffer)
    HAL_UART_Receive_DMA(&huart2, RxBuffer, PKT_LENGTH);
    // 注意：这里长度直接设为 PKT_LENGTH (10)，这样收满10个会自动触发 DMA 中断(虽然我们主要用IDLE)
    // 但为了保险，还是配合 IDLE 中断使用
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/* --- 极简 UART 中断处理 --- */
void Robot_UART_IDLE_Handler(UART_HandleTypeDef *huart) {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {

        // 1. 清除标志位 & 停止 DMA
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);

        // 2. 检查接收长度
        // CNDTR 是倒计数的。如果发了 10 个字节，PKT_LENGTH - CNDTR 应该等于 10
        // 但为了容错（比如只收到了 10 个），我们只关心是否 >= 10
        // 由于我们设置 DMA 请求长度就是 PKT_LENGTH，通常中断时 CNDTR 可能是 0

        // 简单粗暴逻辑：直接校验 RxBuffer 里的数据
        // 因为 DMA 是从 RxBuffer[0] 开始写的

        if (protocol_validate_packet(RxBuffer)) {
            // 校验通过，直接强转解析
            Packet_t *pkt = (Packet_t*)RxBuffer;

            if (pkt->type == PKT_TYPE_CHASSIS) {
                // --- 底盘控制 ---
                Robot.Remote.KeyMask = pkt->data[0];
                Robot.Remote.LastPacketTick = HAL_GetTick();
            }
            else if (pkt->type == PKT_TYPE_ARM) {
                // --- 机械臂转发 ---
                // 直接阻塞转发这 10 个字节 (耗时 < 1ms)
                HAL_UART_Transmit(&huart3, RxBuffer, PKT_LENGTH, 10);
            }
        }

        // 3. 无论成功失败，重置 DMA，准备接收下一包
        // 这会将 buffer 指针指回 RxBuffer[0]，实现"丢弃旧数据"的效果
        memset(RxBuffer, 0, PKT_LENGTH); // 选做：清空缓存防止残留
        HAL_UART_Receive_DMA(huart, RxBuffer, PKT_LENGTH);
    }
}
float feed,target,out;//db

/* --- 周期性任务 (保持不变) --- */
void Robot_Periodic_Task(void) {
    // 1. 超时保护
    if (HAL_GetTick() - Robot.Remote.LastPacketTick > SAFETY_TIMEOUT_MS) {
        Robot.Remote.KeyMask = 0;
    }

    // 2. 运动学解算
    float vx = 0.0f, vy = 0.0f;
    uint8_t mask = Robot.Remote.KeyMask;

    // 按位判断
    float speed = (mask & BTN_BIT_SHIFT) ? SPEED_RPM_TURBO : SPEED_RPM_NORMAL;
    if (mask & BTN_BIT_W) vx += speed;
    if (mask & BTN_BIT_S) vx -= speed;
    if (mask & BTN_BIT_A) vy -= speed;
    if (mask & BTN_BIT_D) vy += speed;

    Robot.Target_Vx = vx;
    Robot.Target_Vy = vy;

    // 麦轮解算
    float target_lf = vx - vy;
    float target_rf = vx + vy;
    float target_lr = vx + vy;
    float target_rr = vx - vy;


    // 3. PID更新 & 电机执行
    float out_lf=(int32_t)PID_Update(&Robot.Pid_LF, target_lf, bsp_motor_get_rpm(MOTOR_CH_FL));
float out_rf=(int32_t)PID_Update(&Robot.Pid_RF, target_rf, bsp_motor_get_rpm(MOTOR_CH_FR));
float out_lr=(int32_t)PID_Update(&Robot.Pid_LB, target_lr, bsp_motor_get_rpm(MOTOR_CH_RL));
float out_rr=(int32_t)PID_Update(&Robot.Pid_RB, target_rr, bsp_motor_get_rpm(MOTOR_CH_RR));
    bsp_motor_periodic_update(ROBOT_DT);
    // 前左 (LF) 轮
    if (target_lf > 0.1f) {
        out_lf += MOTOR_DEADZONE;
    } else if (target_lf < -0.1f) {
        out_lf -= MOTOR_DEADZONE;
    }

    // 前右 (RF) 轮
    if (target_rf > 0.1f) {
        out_rf += MOTOR_DEADZONE;
    } else if (target_rf < -0.1f) {
        out_rf -= MOTOR_DEADZONE;
    }

    // 后左 (LR) 轮
    if (target_lr > 0.1f) {
        out_lr += MOTOR_DEADZONE;
    } else if (target_lr < -0.1f) {
        out_lr -= MOTOR_DEADZONE;
    }

    // 后右 (RR) 轮
    if (target_rr > 0.1f) {
        out_rr += MOTOR_DEADZONE;
    } else if (target_rr < -0.1f) {
        out_rr -= MOTOR_DEADZONE;
    }

    bsp_motor_set_pwm(MOTOR_CH_FL,out_lf);
    bsp_motor_set_pwm(MOTOR_CH_FR,out_rf);
    bsp_motor_set_pwm(MOTOR_CH_RL,out_lr);
    bsp_motor_set_pwm(MOTOR_CH_RR,out_rr);
    //db
    target=target_rf;
    feed=bsp_motor_get_rpm(MOTOR_CH_FR);
    out = out_rf;
}