#include "stm32f10x.h"
 #include "ADC.h" // 请确保包含你的头文件
 #include "Delay.h"

/**
 * @brief ADC 初始化
 * 配置为：独立模式、单次转换、非扫描
 */
void myADC_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 72M/6 = 12MHz (最大不超过14M)

    // 2. 配置 GPIO (PA0 - PA5)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN; // 模拟输入
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. 配置 ADC 参数
    ADC_DeInit(ADC1); // 复位ADC，确保状态干净
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;       // 【关键】关闭扫描
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE; // 【关键】关闭连续转换
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfChannel = 1;             // 【关键】每次只读1个
    ADC_Init(ADC1, &ADC_InitStruct);

    // 4. 开启 ADC 并校准
    ADC_Cmd(ADC1, ENABLE);

    // 复位校准
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));

    // 开始校准
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    // 注意：初始化函数中不需要启动转换 (SoftwareStartConvCmd)，因为我们要在读取函数里启动
}

/**
 * @brief 读取指定通道的 ADC 值
 * @param ADC_Channel: ADC_Channel_x (0~5)
 */
uint16_t AD_GetValue(uint8_t ADC_Channel) 
{
    // 1. 配置规则组：将目标通道配置到 Rank 1
    // 采样时间建议设大一点 (55或239)，防止通道切换时的信号串扰
    ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_55Cycles5);

    // 2. 启动转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // 3. 等待转换结束 (EOC)
    // 建议加上超时机制防止死循环（虽然在正常硬件下极少发生）
    uint16_t timeout = 10000;
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
        timeout--;
        if(timeout == 0) return 0; // 超时返回0
    }

    // 4. 读取数据 (读取DR寄存器会自动清除EOC标志)
    return ADC_GetConversionValue(ADC1);
}

/**
 * @brief 滤波函数
 * 注意：冒泡排序200次浮点数非常耗时，可能导致机械臂卡顿。
 * 建议减小采样次数（例如20-50次）
 */
float filter(uint8_t ADC_Channel) 
{ 
    int count, i, j; 
    // 减小缓冲区大小以提高响应速度，200次对于实时控制来说太慢了
    #define FILTER_N 20  
    float Value_buf[FILTER_N]; 
    float sum = 0; 
    float temp;

    // 1. 采集数据
    for(count = 0; count < FILTER_N; count++) 
    { 
        // 这里去掉了 /100*100 的操作，保留原始精度，让映射更丝滑
        Value_buf[count] = AD_GetValue(ADC_Channel);
    }

    // 2. 排序 (简单的冒泡排序)
    for(j = 0; j < (FILTER_N - 1); j++) 
    { 
        for(i = 0; i < (FILTER_N - j - 1); i++) 
        { 
            if(Value_buf[i] > Value_buf[i+1]) 
            { 
                temp = Value_buf[i]; 
                Value_buf[i] = Value_buf[i+1]; 
                Value_buf[i+1] = temp; 
            } 
        }   
    }
    
    // 3. 去掉最大值和最小值，取平均
    // 从索引 1 开始，到 N-2 结束
    for(count = 1; count < FILTER_N - 1; count++) 
    { 
        sum += Value_buf[count]; 
    } 

    return sum / (FILTER_N - 2);
}