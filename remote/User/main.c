#include "stm32f10x.h"
#include "protocol.h"
#include"ADC.h"
/* --- 外部函数声明 --- */
extern void Remote_Init(void);
extern void Remote_Task(void);

/* --- 调试控制变量 --- */
// 1 = 开启自动循环调试 (忽略按键), 0 = 关闭调试 (读取真实按键)
//uint8_t debug_mask_value = 0; // 用于存放调试生成的按键值

/* --- GPIO 定义 (供 main 初始化使用) --- */
#define BTN_W_PORT     GPIOA
#define BTN_W_PIN      GPIO_Pin_7
#define BTN_A_PORT     GPIOA
#define BTN_A_PIN      GPIO_Pin_6
#define BTN_S_PORT     GPIOB
#define BTN_S_PIN      GPIO_Pin_0
#define BTN_D_PORT     GPIOB
#define BTN_D_PIN      GPIO_Pin_1
#define BTN_SHIFT_PORT GPIOB
#define BTN_SHIFT_PIN  GPIO_Pin_10
#define BTN_CTRL_PORT  GPIOB
#define BTN_CTRL_PIN   GPIO_Pin_11
#define BTN_K5_PORT    GPIOB
#define BTN_K5_PIN     GPIO_Pin_2

/* --- 全局变量 --- */
static volatile uint32_t msTi = 0;

/* --- 时间管理 --- */
void TIM_Config(TIM_TypeDef* TIMx, uint16_t AutoReload, uint16_t Prescaler) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    if (TIMx == TIM3) RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = AutoReload - 1;      
    TIM_TimeBaseStructure.TIM_Prescaler = Prescaler - 1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIMx, ENABLE);
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        msTi++;
    }
}

uint32_t GetTick(void) { return msTi; }

/* --- GPIO 初始化 --- */
void GPIO_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 1. 串口
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 2. 电位器 (暂时不管，但保持模拟输入以免干扰)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 按键组 A (PA6, PA7) - 上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. 按键组 B (PB0, PB1, PB2, PB10, PB11) - 上拉输入
    // 修复：之前漏掉了 GPIO_Pin_2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_10 | GPIO_Pin_11|GPIO_Pin_7| GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void USART1_DMA_Configuration(void) {
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0; 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
}

void USART1_Init(void) {
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    USART1_DMA_Configuration();
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_DMA_Transmit(uint8_t *pBuffer, uint16_t Size) {
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CMAR = (uint32_t)pBuffer;
    DMA1_Channel4->CNDTR = Size;
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

/* --- 主函数 --- */
TIM_Config(TIM3, 72, 1000); 
GPIO_Configuration(); 
USART1_Init();
myADC_Init();

int main(void) { 
    uint32_t last_time = 0;
    Remote_Init();

    while (1) {
        Remote_Task(); 
    }
}