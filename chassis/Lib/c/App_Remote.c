/* --- START OF FILE App_Remote.c (SPL Version) --- */

#include "stm32f10x.h"
#include "protocol.h" 
#include <string.h>

/* === Configuration === */
#define SEND_PERIOD_CHASSIS_MS   20U   /* 50Hz */
#define SEND_PERIOD_ARM_MS       50U   /* 20Hz */

/* === GPIO Definitions (SPL Style) === */
// 请确保在 main.c 中已经开启了 GPIOA 时钟并配置为 GPIO_Mode_IPD 或 GPIO_Mode_IPU
#define BTN_W_PORT    GPIOA
#define BTN_W_PIN     GPIO_Pin_0
#define BTN_A_PORT    GPIOA
#define BTN_A_PIN     GPIO_Pin_1
#define BTN_S_PORT    GPIOA
#define BTN_S_PIN     GPIO_Pin_2
#define BTN_D_PORT    GPIOA
#define BTN_D_PIN     GPIO_Pin_3
#define BTN_SHIFT_PORT GPIOA
#define BTN_SHIFT_PIN  GPIO_Pin_4

/* 
 * 外部依赖：
 * SPL 没有 HAL_GetTick()，你需要自己实现一个返回系统运行毫秒数的函数。
 * 这里假设函数名为 GetTick()。
 */
extern uint32_t GetTick(void); 

/* Internal State */
static uint32_t last_chassis_tick = 0;
static uint32_t last_arm_tick = 0;

/* Data Storage */
static uint8_t s_arm_joints[6] = {150, 150, 150, 150, 150, 150}; // Default neutral

/* 
 * DMA Buffer 
 * 必须是全局或静态变量，因为 DMA 发送是异步的
 */
Packet_t tx_packet_buffer;

/* --- Private Functions --- */

static uint8_t Remote_ReadButtonMask(void)
{
    Remote_State_t st;
    st.all = 0;
    st.bits.W=1;
    // SPL 读取输入引脚: GPIO_ReadInputDataBit 返回 Bit_SET (1) 或 Bit_RESET (0)
  /*  st.bits.W     = (GPIO_ReadInputDataBit(BTN_W_PORT, BTN_W_PIN) == Bit_SET) ? 1 : 0;
    st.bits.A     = (GPIO_ReadInputDataBit(BTN_A_PORT, BTN_A_PIN) == Bit_SET) ? 1 : 0;
    st.bits.S     = (GPIO_ReadInputDataBit(BTN_S_PORT, BTN_S_PIN) == Bit_SET) ? 1 : 0;
    st.bits.D     = (GPIO_ReadInputDataBit(BTN_D_PORT, BTN_D_PIN) == Bit_SET) ? 1 : 0;
    st.bits.Shift = (GPIO_ReadInputDataBit(BTN_SHIFT_PORT, BTN_SHIFT_PIN) == Bit_SET) ? 1 : 0;
 */   
    return st.all;
}

/**
 * @brief SPL 版本 DMA 发送辅助函数
 * 对于 STM32F103，USART1_TX 对应 DMA1_Channel4
 */
static void Remote_DMA_Transmit(uint8_t *data, uint16_t len)
{
    // 1. 关闭 DMA 通道以进行配置 (如果未关闭，设置 CNDTR 无效)
    DMA_Cmd(DMA1_Channel4, DISABLE);
    
    // 2. 设置传输地址 (如果初始化时已设置且 buffer 地址不变，这一步可省略，但为了保险加上)
    DMA1_Channel4->CMAR = (uint32_t)data;
    
    // 3. 设置传输数据量
    DMA1_Channel4->CNDTR = len;
    
    // 4. 清除传输完成标志 (防止误判)
    DMA_ClearFlag(DMA1_FLAG_TC4);
    
    // 5. 开启 DMA 通道，开始发送
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

/* --- Public API --- */

void Remote_Init(void)
{
    last_chassis_tick = GetTick();
    last_arm_tick = GetTick();
    
    // 注意：需要在 main.c 中完成 GPIO, USART1 和 DMA1_Channel4 的硬件初始化
}

/**
 * Call this to update arm data from ADC/Potentiometers
 */
void Remote_UpdateArmData(uint8_t joint_index, uint8_t value)
{
    if (joint_index < 6) {
        if (value < 50) value = 50;
        if (value > 250) value = 250;
        s_arm_joints[joint_index] = value;
    }
}

void Remote_Task(void)
{
    uint32_t now = GetTick();

    // === Task 1: Chassis Control (High Priority, ~20ms) ===
    if (now - last_chassis_tick >= SEND_PERIOD_CHASSIS_MS) {
        last_chassis_tick = now;

        // 检查 DMA 是否忙碌 (检查 CNDTR 寄存器是否为0，或者检查 TC 标志)
        // 如果 CNDTR > 0，说明上一次发送还没结束
        if (DMA_GetCurrDataCounter(DMA1_Channel4) == 0) {
            uint8_t mask = Remote_ReadButtonMask();

            // 填充缓冲区
            pack_chassis(&tx_packet_buffer, mask);

            // 通过 DMA 发送
            Remote_DMA_Transmit((uint8_t*)&tx_packet_buffer, PKT_LENGTH);

            // 本次循环已发送，跳过 Arm 检查以防冲突
            return;
        }
    }

    // === Task 2: Arm Control (Lower Priority, ~50ms) ===
    if (now - last_arm_tick >= SEND_PERIOD_ARM_MS) {
        last_arm_tick = now;

        // 检查 DMA 是否忙碌
        if (DMA_GetCurrDataCounter(DMA1_Channel4) == 0) {
            // 填充缓冲区
            pack_arm(&tx_packet_buffer, s_arm_joints);

            // 通过 DMA 发送
            Remote_DMA_Transmit((uint8_t*)&tx_packet_buffer, PKT_LENGTH);
        }
    }
}

/* --- END OF FILE App_Remote.c --- */