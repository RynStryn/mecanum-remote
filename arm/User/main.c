/* --- START OF FILE main.c 修改版 --- */
#include "stm32f10x.h"
#include "Serial.h"
#include "TIM.h"
#include "protocol_arm.h" // 确保这里面有 PKT_HEAD, PKT_TAIL 等定义
#include <string.h>

// --- 配置 ---
// 机械臂协议长度是10，建议缓冲区稍微大一点，防止溢出
#define RX_BUF_SIZE  64  
#define PKT_LEN      10

// 全局调试变量
volatile uint8_t g_last_rx_buf[RX_BUF_SIZE] = {0}; 
volatile uint16_t g_last_rx_len = 0;
volatile uint32_t g_crc_fail_cnt = 0; // 校验失败
volatile uint32_t g_pkt_cnt = 0;      // 成功接收

/**
 * @brief 映射函数 (保持不变)
 */
uint16_t map_servo_value(uint8_t byte_value)
{
    uint16_t pulse;
    // 限制范围，防止舵机卡死
    if (byte_value < 50) byte_value = 50;
    else if (byte_value > 250) byte_value = 250;
    
    pulse = SERVO_MIN_PULSE + (uint16_t)(byte_value - 50) * 10;
    return pulse;
}

/**
 * @brief 简单的异或校验计算
 */
uint8_t calc_checksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    // 协议：Checksum = Type ^ Data[0] ^ ... ^ Data[5]
    // 这里的 data 指针应当指向 Type (即 buffer[1])
    for(int i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

int main(void) 
{
    // 系统初始化
    // 务必确保 SystemInit 在 startup 文件中被调用，或在这里调用
    // SystemInit(); 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    USART1_Init();      // 确保内部 DMA 初始化长度为 RX_BUF_SIZE
    TIM_Servo_Init();   

    while(1) 
    {
        // 可以在这里加一个 LED 闪烁，证明主循环在跑，没有死机
    }
}

/**
 * @brief USART1 中断服务函数 - 修复版
 */
void USART1_IRQHandler(void)
{
    // 检查 IDLE 标志
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t temp;
        
        // 1. 关闭 DMA，防止数据在处理时继续写入导致混乱
        DMA_Cmd(DMA1_Channel5, DISABLE); 

        // 2. 清除 IDLE 标志 
        temp = USART1->SR;
        temp = USART1->DR; 
        (void)temp; 

        // 3. 计算接收到的数据长度
        uint16_t remain = DMA_GetCurrDataCounter(DMA1_Channel5);
        uint16_t received_len = RX_BUF_SIZE - remain;

        // 4. 【调试快照】方便在 Debugger 中观察
        if (received_len > 0 && received_len <= RX_BUF_SIZE) {
            g_last_rx_len = received_len;
            memcpy((void*)g_last_rx_buf, usart1_rx_buf, received_len);
        }

        // 5. 滑动窗口解析
        uint16_t cursor = 0;
        
        // 只要剩余数据够一个包长，就继续尝试解析
        while (cursor + PKT_LEN <= received_len) 
        {
            // A. 匹配包头 (0xFF) 和 包尾 (0xFE)
            if (usart1_rx_buf[cursor] == PKT_HEAD && 
                usart1_rx_buf[cursor + 9] == PKT_TAIL)
            {
                // B. 校验计算 
                uint8_t *pType = &usart1_rx_buf[cursor + 1];
                uint8_t calculated_sum = calc_checksum(pType, 7); // Type(1) + Data(6)
                uint8_t received_sum = usart1_rx_buf[cursor + 8];

                if (calculated_sum == received_sum) 
                {
                    //校验通过
                    g_pkt_cnt++;
                    
                    // C. 提取舵机数据并执行
                    uint16_t pulse[6];
                    for(int i = 0; i < 6; i++) {
                        pulse[i] = map_servo_value(usart1_rx_buf[cursor + 2 + i]);
                    }
                    
                    // D. 更新 PWM
                    TIM_SetCompare1(TIM2, pulse[0]); 
                    TIM_SetCompare2(TIM2, pulse[1]); 
                    TIM_SetCompare3(TIM2, pulse[2]); 
                    TIM_SetCompare4(TIM2, pulse[3]); 
                    TIM_SetCompare1(TIM3, pulse[4]); 
                    TIM_SetCompare2(TIM3, pulse[5]); 
                    
                    // E. 解析成功后，跳过这一个包，寻找下一个
                    cursor += PKT_LEN;
                    continue; 
                }
                else {
                    g_crc_fail_cnt++; // 校验错误
                }
            }
            
            // 如果不是包头，或者校验失败：
            // 光标仅后移 1 位，尝试下一位是不是包头
            cursor++;
        }
        
        // 6. 重启 DMA 接收下一批数据
        // 顺序：Disable(已做) -> 设置数量 -> 清除标志 -> Enable
        
        // 重置接收数量为最大缓冲区大小
        DMA_SetCurrDataCounter(DMA1_Channel5, RX_BUF_SIZE);
        
        // 清除所有 DMA 标志位 (传输完成、半传输、错误)
        DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_HT5 | DMA1_FLAG_GL5 | DMA1_FLAG_TE5);
        
        // 开启 DMA
        DMA_Cmd(DMA1_Channel5, ENABLE);
        
        // *** 绝对不要在这里再次读取 USART1->DR ***
        // 否则会吞掉刚进来的新数据字节
    }
}