#include "App_RemoteSim.h"
#include "protocol.h"

/* === 外部 UART 句柄 === */
extern UART_HandleTypeDef huart2;

/* === 配置参数 === */
#define SIM_PERIOD_CHASSIS_MS   20   // 50Hz
#define SIM_PERIOD_ARM_MS       100  // 10Hz

/* === 私有状态 === */
static uint32_t last_chassis_tick = 0;
static uint32_t last_arm_tick = 0;

static uint8_t  sim_btn_mask = 0;
static uint8_t  sim_arm_data[6] = {150, 150, 150, 150, 150, 150}; // 默认中位

/* === 初始化 === */
void RemoteSim_Init(void)
{
    last_chassis_tick = HAL_GetTick();
    last_arm_tick = HAL_GetTick();
    sim_btn_mask = 0;
}

/* === API: 设置模拟按键 === */
void RemoteSim_SetButtons(uint8_t mask)
{
    sim_btn_mask = mask;
}

/* === API: 设置模拟机械臂数据 === */
void RemoteSim_SetArm(uint8_t joint_idx, uint8_t value)
{
    if (joint_idx < 6) {
        // 简单的安全限幅
        if (value < 50) value = 50;
        if (value > 250) value = 250;
        sim_arm_data[joint_idx] = value;
    }
}

/* === 周期性任务 === */
void RemoteSim_Run(void)
{
    uint32_t now = HAL_GetTick();

    // --- 任务 1: 底盘模拟 (每20ms发送一次) ---
    if (now - last_chassis_tick >= SIM_PERIOD_CHASSIS_MS) {
        last_chassis_tick = now;

        Packet_t pkt;
        pack_chassis(&pkt, sim_btn_mask);

        // 模拟器环境使用阻塞发送最简单且安全 (10字节耗时 < 1ms)
        HAL_UART_Transmit(&huart2, (uint8_t*)&pkt, PKT_LENGTH, 10);
    }

    // --- 任务 2: 机械臂模拟 (每100ms发送一次) ---
    if (now - last_arm_tick >= SIM_PERIOD_ARM_MS) {
        last_arm_tick = now;

        Packet_t pkt;
        pack_arm(&pkt, sim_arm_data);

        // 阻塞发送
        HAL_UART_Transmit(&huart2, (uint8_t*)&pkt, PKT_LENGTH, 10);
    }
}