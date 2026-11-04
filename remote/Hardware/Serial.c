#include "Serial.h"


// USART1初始化（PA9-TX, PA10-RX）

// 单字节发送
void USART_SendByte(USART_TypeDef* USARTx, uint8_t dat) 
{
    USART_SendData(USARTx, dat);
    while(!USART_GetFlagStatus(USARTx, USART_FLAG_TXE));
}

// 缓冲区发送
void USART_SendBuf(USART_TypeDef* USARTx, uint8_t *buf, uint8_t len) 
{
    for(uint8_t i=0; i<len; i++) 
		{
        USART_SendByte(USARTx, buf[i]);
    }
}

void Serial_SendString(char*string)
{
	uint8_t i;
	for(i=0;string[i]!=0;i++)
	{
		USART_SendByte(USART1,string[i]);
	}
}

//void ESP8266_Init(void)
//{
//	Serial_SendString("AT+RESTORE\r\n");
//	Delay_ms (50);
//	Serial_SendString("AT+CWMODE=1\r\n");
//	Delay_ms (50);
//	Serial_SendString("AT+CWJAP=\"ESP_AP\",\"12345678\"\r\n");
//	Delay_ms (50);
//	Serial_SendString("AT+CIPMUX=0\r\n");
//	Delay_ms (50);
//	Serial_SendString("AT+CIPSTART=\"TCP\",\"192.168.4.1\",8080\r\n");
//	Delay_ms (50);
//	Serial_SendString("AT+CIPMPDE=1\r\n");
//	Delay_ms (50);
//	Serial_SendString("AT+CIPSEND\r\n");
//	Delay_ms (50);
//}

