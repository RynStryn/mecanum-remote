#include "Timer.h"

void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM2 ,ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period =10;
	TIM_TimeBaseInitStructure.TIM_Prescaler =71999;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter =0x00;
	TIM_TimeBaseInit (TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig (TIM2,TIM_IT_Update,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd (TIM2,ENABLE);
}



