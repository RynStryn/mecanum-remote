#ifndef __SERIAL_H
#define __SERIAL_H
#include "stm32f10x.h"
#include "Delay.h"
#include <string.h>

#define USART_BAUD 115200


// 初始化函数
void USART1_Init(void);  // PA9-TX, PA10-RX


// 发送函数
void USART_SendByte(USART_TypeDef* USARTx, u8 dat);
void USART_SendBuf(USART_TypeDef* USARTx, u8 *buf, u8 len);
void ESP8266_Init(void);
#endif
