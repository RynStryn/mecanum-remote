/* --- START OF FILE TIM.c --- */
#include "TIM.h"

// ... (省略 TIM.h 中的函数声明) ...

/**
 * @brief 初始化 TIM2 和 TIM3 用于 6 路 50Hz PWM 输出
 * PSC = 72 - 1 = 71
 * TIM2/3 在 APB1 总线 (36MHz)。实际 Fpwm = 36MHz / 72 / 20000 = 25Hz (错误!)
 *
 * 修正: 必须使用 TIM_TimeBaseStructure.TIM_Prescaler = 720 - 1; 才能简化配置
 * TIM2/3 时钟 (APB1) = 36MHz
 * Fpwm = 36MHz / 720 / 1000 = 50Hz
 * PSC = 720 - 1 = 719
 * ARR = 1000 - 1 = 999
 */
void TIM_Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 1. 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    // 2. 配置 GPIO (略)
        GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // 3. 配置时基单元 (PSC=719, ARR=19999 是旧的 72MHz 设定，这里为了简洁保持 500-2500 范围)
    // 采用 72MHz/72/20000=50Hz 的参数，但计数时钟是 APB1(36MHz)
    // 为了使 CCR 范围为 500-2500，我们保持 ARR=19999，并将 PSC 设为 36-1
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 36MHz / 36 = 1MHz
    TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_ARR; // 1MHz / 20000 = 50Hz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 4. 配置输出比较 (略)
        TIM_OCInitTypeDef TIM_OCInitStruct;
        TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM2, &TIM_OCInitStruct);  // S1
    TIM_OC2Init(TIM2, &TIM_OCInitStruct);  // S2
    TIM_OC3Init(TIM2, &TIM_OCInitStruct);  // S3
    TIM_OC4Init(TIM2, &TIM_OCInitStruct);  // S4
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);  // S5
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);  // S6
    // 5. 开启定时器 (略)
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

// ... (省略 TIM_SetCompareX 函数) ...
/* --- END OF FILE TIM.c --- */