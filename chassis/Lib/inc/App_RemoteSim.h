#ifndef APP_REMOTE_SIM_H
#define APP_REMOTE_SIM_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 初始化 */
void RemoteSim_Init(void);

/* 周期性运行 (放在 main while(1) 中) */
void RemoteSim_Run(void);

/* 设置模拟器的按键状态 */
void RemoteSim_SetButtons(uint8_t btn_mask);

/* 设置模拟器的机械臂关节 (idx: 0-5, value: 50-250) */
void RemoteSim_SetArm(uint8_t joint_idx, uint8_t value);

#endif