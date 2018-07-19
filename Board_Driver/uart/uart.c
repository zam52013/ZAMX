/****************************************************************************
 *@file uart.c
 *   Copyright (c) 2018-2018, 2018 feima Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 * @author zam
 * @email zhangam@feimarobotics.com
 * @date 2018-07-03
 * 
 * 
 ****************************************************************************/
 
 /* Includes ------------------------------------------------------------------*/
 #include "uart.h"
#include <string.h>
#include <stdlib.h> 
#include <stdio.h> 


#pragma import(__use_no_semihosting)              
 //标准库需要的支持函数                  
 struct __FILE  
 {  
 	int handle;  
 };  
 
 
 FILE __stdout;        
 //定义_sys_exit()以避免使用半主机模式     
 int _sys_exit(int x)  
 {  
 	x = x;  
 }  
 
 //重定义fputc函数  
 int fputc(int ch, FILE *f) 
 {       
 		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕    
     		USART1->DR = (u8) ch;       
 		return ch; 
 } 

 void UARTx_Init(USART_Driver* USARTx)
 {
	 uint8_t UART_TI_CNT=0;
	GPIO_InitTypeDef GPIO_InitStructure; 
 	USART_InitTypeDef USART_InitStructure; 
	 #ifdef USARTx_USE_DMA
	DMA_InitTypeDef DMA_InitStructure;
	#endif
	USARTx->USART_CLK(USARTx->USART_Func, ENABLE);
	USARTx->TX_GPIOClk(USARTx->TX_GPIOFunc, ENABLE);
	USARTx->RX_GPIOClk(USARTx->RX_GPIOFunc, ENABLE);

	GPIO_PinAFConfig(USARTx->TX_GPIO, USARTx->TX_Src, USARTx->GPIO_AF_USART);
	GPIO_PinAFConfig(USARTx->RX_GPIO, USARTx->RX_Src, USARTx->GPIO_AF_USART);
  
	GPIO_StructInit(&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin = USARTx->TX_Pin;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(USARTx->TX_GPIO, &GPIO_InitStructure);
  
  	GPIO_InitStructure.GPIO_Pin = USARTx->RX_Pin;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
 	GPIO_Init(USARTx->RX_GPIO, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = USARTx->InitTypeDef.USART_BaudRate;
  	USART_InitStructure.USART_WordLength = USARTx->InitTypeDef.USART_WordLength;
  	USART_InitStructure.USART_StopBits = USARTx->InitTypeDef.USART_StopBits;
  	USART_InitStructure.USART_Parity = USARTx->InitTypeDef.USART_Parity;
  	USART_InitStructure.USART_HardwareFlowControl = USARTx->InitTypeDef.USART_HardwareFlowControl;
  	USART_InitStructure.USART_Mode = USARTx->InitTypeDef.USART_Mode;
	USART_Init(USARTx->USART, &USART_InitStructure);
	for(UART_TI_CNT=0;UART_TI_CNT<4;UART_TI_CNT++)
	{
		switch(USARTx->NVIC_FLAG[UART_TI_CNT])
		{
			case USART_IT_CTS:
				USART_ITConfig(USARTx->USART, USART_IT_CTS, ENABLE);
				USART_ClearFlag(USARTx->USART, USART_IT_CTS);
				break;
			case USART_IT_LBD:
				USART_ITConfig(USARTx->USART, USART_IT_LBD, ENABLE);
				USART_ClearFlag(USARTx->USART, USART_IT_LBD);
				break;
			case USART_IT_TXE:
				USART_ITConfig(USARTx->USART, USART_IT_TXE, ENABLE);
				break;
			case USART_IT_TC:
				USART_ITConfig(USARTx->USART, USART_IT_TC, ENABLE);
				USART_ClearFlag(USARTx->USART, USART_IT_TC);
				break;
			case USART_IT_RXNE:
				USART_ITConfig(USARTx->USART, USART_IT_RXNE, ENABLE);
				USART_ClearFlag(USARTx->USART, USART_IT_RXNE);	
				break;
			case USART_IT_IDLE:
				USART_ITConfig(USARTx->USART, USART_IT_IDLE, ENABLE);
				break;
			case USART_IT_PE:
				USART_ITConfig(USARTx->USART, USART_IT_PE, ENABLE);
				break;
			case USART_IT_ERR:
				USART_ITConfig(USARTx->USART, USART_IT_ERR, ENABLE);
				break;
			default:
				break;
		}
	}
	NVIC_Init(&USARTx->NVIC_USART);	

	#ifdef USARTx_USE_DMA
	if(USARTx->USE_DMA)
	{
		NVIC_Init(&USARTx->NVIC_DMA_TX);
		
		DMA_DeInit(USARTx->DMA_TX_Stream);
		DMA_InitStructure.DMA_Channel = USARTx->DMA_TX_CH;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTx->USART->DR));
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USARTx->DMA_TX_Buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = USARTx->DMA_TX_Size;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(USARTx->DMA_TX_Stream, &DMA_InitStructure);
		DMA_ITConfig(USARTx->DMA_TX_Stream, USARTx->NVIC_TI_FLAG, ENABLE);

		DMA_DeInit(USARTx->DMA_RX_Stream);
		DMA_InitStructure.DMA_BufferSize = USARTx->DMA_RX_Size;
			DMA_InitStructure.DMA_Channel = USARTx->DMA_RX_CH;;
		 DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
			DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)USARTx->DMA_RX_Buffer; 
			DMA_Init(USARTx->DMA_RX_Stream, &DMA_InitStructure);
		DMA_Cmd(USARTx->DMA_RX_Stream, ENABLE);
		USART_DMACmd(USARTx->USART, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
	}
	#endif
	USART_Cmd(USARTx->USART, ENABLE);   
 }


void USARTx_DeInit(USART_Driver* USARTx)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  	GPIO_InitStructure.GPIO_Pin = USARTx->TX_Pin;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(USARTx->TX_GPIO, &GPIO_InitStructure);
	
  	GPIO_InitStructure.GPIO_Pin = USARTx->RX_Pin;
  	GPIO_Init(USARTx->RX_GPIO, &GPIO_InitStructure);

	#ifdef USARTx_USE_DMA
	if(USARTx->USE_DMA)
	{
		DMA_DeInit(USARTx->DMA_TX_Stream);
		while (DMA_GetCmdStatus(USARTx->DMA_TX_Stream) != DISABLE);
		DMA_Cmd(USARTx->DMA_TX_Stream, DISABLE);
		DMA_DeInit(USARTx->DMA_RX_Stream);
		while (DMA_GetCmdStatus(USARTx->DMA_RX_Stream) != DISABLE);
		DMA_Cmd(USARTx->DMA_RX_Stream, DISABLE);
	}
	#endif
}

void USARTx_SendByte(USART_Driver* USARTx, uint8_t byte)
{
	USART_SendData(USARTx->USART, byte);
  	while (USART_GetFlagStatus(USARTx->USART, USART_FLAG_TXE) == RESET);
}

void USARTx_SendBytes(USART_Driver* USARTx, uint8_t* buffer, uint16_t length)
{
	uint16_t i = 0;
	
	while(i < length)
	{
		while (USART_GetFlagStatus(USARTx->USART, USART_FLAG_TC) == RESET);
		USART_SendData(USARTx->USART, buffer[i]);
		i++;
	}
}

void UART_SendString(USART_Driver* USARTx,char* s)
{
	while(*s !='\0')//检测字符串结束符
	{
		USART_SendData(USARTx->USART ,*s);//发送当前字符
		while(USART_GetFlagStatus(USARTx->USART, USART_FLAG_TXE)==RESET); 
		s++;
	}
}

	
	/**
  * @
  */ 

/**
  * @
  */

/**
  * @
  */ 

/**
  * @
  */ 

/************************ (C) COPYRIGHT feima *****END OF FILE****/	
	
	
	
	
	
	
