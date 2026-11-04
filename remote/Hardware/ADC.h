#ifndef __ADC_H
#define __ADC_H
#include "stm32f10x.h"

void myADC_Init(void);  // ��ʼ��ADC
float filter(uint8_t ADC_Channel);  //�˲��㷨

#endif
