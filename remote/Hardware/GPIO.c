#include "GPIO.h"
uint8_t W;
uint8_t A;
uint8_t S;
uint8_t D;
uint8_t Shift;
uint8_t Wheel_Pack;
void Key_Init(void)
{
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA ,ENABLE);
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB ,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitTypeStructure;
	GPIO_InitTypeStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitTypeStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitTypeStructure);
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_Init(GPIOB,&GPIO_InitTypeStructure);
}


uint8_t Key_Pack(void)
{
	W=GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_6);
	A=GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_7);
	S=GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_7);
	D=GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_7);
	Shift=GPIO_ReadInputDataBit (GPIOA,GPIO_Pin_7);
	Wheel_Pack=(Shift<<4|D<<3|S<<2|A<<1|W)&0xFF;
	return Wheel_Pack;
}

