#include "../inc/App_Robot.h"
#include <string.h>
#include <math.h>
/* 调试计数器 */
volatile uint32_t Cnt_ISR_Enter = 0;   // 进了多少次中断
volatile uint32_t Cnt_CRC_Fail = 0;    // 校验失败多少次
volatile uint32_t Cnt_Type_Arm = 0;    // 识别出 Arm 包次数
volatile uint32_t Cnt_Type_Chassis = 0;// 识别出 Chassis 包次数
volatile uint32_t Cnt_ORE_Error = 0;   // 溢出错误次数

/*  调试快照  */
volatile uint32_t Debug_Last_Len = 0;      // 最后一次收到的长度
volatile uint8_t  Debug_Last_Buffer[20];   // 最后一次收到的原始数据
volatile uint8_t  Debug_Last_Type = 0;     // 解析出的类型
volatile uint8_t  Debug_Check_Result = 0;  // 校验函数返回的结果


Robot_TypeDef Robot;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define PACKET_QUEUE_SIZE 64  // 专门给机械臂用的队列
#define DMA_BUF_SIZE  50
/* --- 1. 机械臂专用 FIFO --- */
// 只有 Arm 的包会进这里
static Packet_t Arm_FIFO[PACKET_QUEUE_SIZE];
static volatile uint16_t FIFO_Head = 0;
static volatile uint16_t FIFO_Tail = 0;
static uint8_t Uart3_TxBuffer[10];
/* --- 2. 接收缓冲池 --- */
static uint8_t RxRaw_Pool[2][DMA_BUF_SIZE];
static volatile uint8_t RxRaw_Index = 0;

/* --- 初始化  --- */
void Robot_Init(void) {
    memset(&Robot, 0, sizeof(Robot_TypeDef));
    bsp_motor_init();

    // PID 参数初始化 (略)
    float kp =KP, ki = KI, kd = KD;
    PID_Init(&Robot.Pid_LF, kp, ki, kd, ROBOT_DT, 3500, 7199);
    PID_Init(&Robot.Pid_RF, kp, ki, kd, ROBOT_DT, 3500, 7199);
    PID_Init(&Robot.Pid_LB, kp, ki, kd, ROBOT_DT, 3500, 7199);
    PID_Init(&Robot.Pid_RB, kp, ki, kd, ROBOT_DT, 3500, 7199);

    FIFO_Head = 0; FIFO_Tail = 0;
    RxRaw_Index = 0;
    HAL_UART_Receive_DMA(&huart2, RxRaw_Pool[RxRaw_Index], DMA_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/* --- UART 中断处理 (分类) --- */
void Robot_UART_IDLE_Handler(UART_HandleTypeDef *huart) {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {

        //清除标志 & 停止 DMA
        volatile uint32_t tmp;
        tmp = huart->Instance->SR;
        tmp = huart->Instance->DR;
        (void)tmp;
        HAL_UART_DMAStop(huart);

        // 计算长度
        uint32_t received_len = DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

        // Debug 记录
        Debug_Last_Len = received_len;

        uint8_t *pRaw = RxRaw_Pool[RxRaw_Index];
        RxRaw_Index = !RxRaw_Index;
        HAL_UART_Receive_DMA(huart, RxRaw_Pool[RxRaw_Index], DMA_BUF_SIZE);

        // 滑动搜索
        uint32_t cursor = 0;

        // 只要剩余数据还够一个包长，就继续找
        while (cursor + PKT_LENGTH <= received_len) {

            // 1. 先判断包头是否匹配
            if (pRaw[cursor] == PKT_HEADER_BYTE) {

                // 2. 只有包头对了，才去校验整个包
                if (protocol_validate_packet(&pRaw[cursor])) {
                    // 解析成功,raw升级为正式pkt
                    Packet_t *pkt = (Packet_t*)&pRaw[cursor];

                    if (pkt->type == PKT_TYPE_CHASSIS) {
                        Cnt_Type_Chassis++;
                        Robot.Remote.KeyMask = pkt->data[0];
                        Robot.Remote.LastPacketTick = HAL_GetTick();
                    }
                    else if (pkt->type == PKT_TYPE_ARM) {
                        Cnt_Type_Arm++;
                        uint16_t next_head = (FIFO_Head + 1) % PACKET_QUEUE_SIZE;
                        if (next_head != FIFO_Tail) {
                            memcpy((void*)&Arm_FIFO[FIFO_Head], pkt, PKT_LENGTH);
                            FIFO_Head = next_head;
                        }
                    }

                    // 处理完这一包，光标跳过
                    cursor += PKT_LENGTH;
                    continue;
                }
            }

            // 3. 如果不是包头，或者校验失败：
            // 光标只往后移 1 位，尝试下一位是不是包头
            cursor++;
        }
    }
}
void Robot_Protocol_Process(void) {
    // 1. 队列检查
    if (FIFO_Head == FIFO_Tail) return;

    // 2. 检查 UART3 DMA 是否忙
    // 如果上一次还没发完，这次就先别发了，防止冲突
    if (huart3.gState != HAL_UART_STATE_READY) {
        return;
    }

    // 3. 取出最新包
    uint16_t temp_head = FIFO_Head;
    uint16_t latest_idx = (temp_head - 1 + PACKET_QUEUE_SIZE) % PACKET_QUEUE_SIZE;
    Packet_t *pkt = &Arm_FIFO[latest_idx];

    // 4. 清空队列
    FIFO_Tail = temp_head;

    // 5. 数据拷贝到专用发送区
    //  pkt 指向的 FIFO 可能会被修改
    memcpy(Uart3_TxBuffer, (uint8_t*)pkt, PKT_LENGTH);

    // 6. 使用 DMA 发送
    // 这样发送过程不会被中断打断，保证 10 字节连续
    HAL_UART_Transmit_DMA(&huart3, Uart3_TxBuffer, PKT_LENGTH);
}

static inline float apply_deadzone(float val) {
    if (val > 0.1f) return val + MOTOR_DEADZONE;
    if (val < -0.1f) return val - MOTOR_DEADZONE;
    return 0.0f;
}
/* --- 周期性任务 --- */
float out;
void Robot_Periodic_Task(void)
{
    // 1. 处理转发队列 (机械臂)
    Robot_Protocol_Process();

    // 1. 获取反馈 (放在最前面，保证 PID 用的是最新数据)
    bsp_motor_periodic_update(ROBOT_DT);

    // 2. 超时保护
    if (HAL_GetTick() - Robot.Remote.LastPacketTick > SAFETY_TIMEOUT_MS) {
        Robot.Remote.KeyMask = 0;
    }

    // 3. 运动学目标解算

    uint8_t mask = Robot.Remote.KeyMask;
    if (mask & BTN_BIT_STOP) {
        Robot.Target_Vx = 0.0f;
        Robot.Target_Vy = 0.0f;

        // 4.1 清空 PID 积分项，防止恢复时猛冲
        PID_Reset(&Robot.Pid_LF);
        PID_Reset(&Robot.Pid_RF);
        PID_Reset(&Robot.Pid_LB);
        PID_Reset(&Robot.Pid_RB);

        // 4.2 物理层立即切断输出 (不经过死区和PID计算)
        bsp_motor_set_pwm(MOTOR_CH_FL, 0);
        bsp_motor_set_pwm(MOTOR_CH_FR, 0);
        bsp_motor_set_pwm(MOTOR_CH_RL, 0);
        bsp_motor_set_pwm(MOTOR_CH_RR, 0);

        return; // 直接结束本次任务
    }
    float vx = 0.0f, vy = 0.0f;
    // 按位判断
    float speed = (mask & BTN_BIT_SHIFT) ? SPEED_RPM_TURBO : SPEED_RPM_NORMAL;
    float rot_speed  = SPEED_RPM_ROT_NORMAL;
    if (mask & BTN_BIT_W) vx += speed;
    if (mask & BTN_BIT_S) vx -= speed;
    if (mask & BTN_BIT_A) vy -= speed;
    if (mask & BTN_BIT_D) vy += speed;

    float omega = 0.0f;
    if (mask & BTN_BIT_E) omega += rot_speed; // E键: 顺时针
    if (mask & BTN_BIT_Q) omega -= rot_speed; // Q键: 逆时针
    Robot.Target_Vx = vx;//调试
    Robot.Target_Vy = vy;
    // 麦轮解算

    float target_lf = vx - vy + omega;
    float target_rf = vx + vy + omega;
    float target_lr = vx + vy - omega;
    float target_rr = vx - vy - omega;

    // 4. PID 计算
    float out_lf = PID_Update(&Robot.Pid_LF, target_lf, bsp_motor_get_rpm(MOTOR_CH_FL));
    float out_rf = PID_Update(&Robot.Pid_RF, target_rf, bsp_motor_get_rpm(MOTOR_CH_FR));
    float out_lr = PID_Update(&Robot.Pid_LB, target_lr, bsp_motor_get_rpm(MOTOR_CH_RL));
    float out_rr = PID_Update(&Robot.Pid_RB, target_rr, bsp_motor_get_rpm(MOTOR_CH_RR));
    if (isnan(out_lf) || isnan(out_rf) || isnan(out_lr) || isnan(out_rr))
    {
        // 发生错误时全部清零并重置
        out_lf = out_rf = out_lr = out_rr = 0.0f;
        PID_Reset(&Robot.Pid_LF);
        PID_Reset(&Robot.Pid_RF);
        PID_Reset(&Robot.Pid_LB);
        PID_Reset(&Robot.Pid_RB);
    }
    // 5. 死区补偿与限幅


    out_lf = apply_deadzone(out_lf);
    out_rf = apply_deadzone(out_rf);
    out_lr = apply_deadzone(out_lr);
    out_rr = apply_deadzone(out_rr);
    out=out_lr;
    // 6. 执行 PWM
    bsp_motor_set_pwm(MOTOR_CH_FL, (int16_t)out_lf);
    bsp_motor_set_pwm(MOTOR_CH_FR, (int16_t)out_rf);
    bsp_motor_set_pwm(MOTOR_CH_RL, (int16_t)out_lr);
    bsp_motor_set_pwm(MOTOR_CH_RR, (int16_t)out_rr);
}