#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"
#include "protocol_arm.h"

// DMA1 Channel 5 for USART1_RX
#define USART1_RX_DMA_CHANNEL DMA1_Channel5

extern uint8_t usart1_rx_buf[50];

void USART1_Init(void);

#endif
