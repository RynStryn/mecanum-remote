#ifndef BSP_DEBUG_H
#define BSP_DEBUG_H

#include "stm32f1xx_hal.h"
#include <stdio.h>

// --- 配置 ---
// 选择用于打印调试信息的串口 (通常是连接电脑的 USB-TTL)
// 注意：不要和 Robot 接收遥控器的串口冲突
#define DEBUG_UART_HANDLE   huart5

// 调试输出频率 (ms) - 不要太快，以免阻塞串口
#define DEBUG_RATE_MS       100

// --- API ---
void Debug_Init(void);
void Debug_Task(void); // 放在 main while(1) 中

#endif // BSP_DEBUG_H