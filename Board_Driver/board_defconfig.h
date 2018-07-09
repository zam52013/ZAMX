/****************************************************************************
 *@file board_defconfig.h
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
 * @date 2018-07-04
 * 
 * 
 ****************************************************************************/
 
 #ifndef __BOARD_DEFCONFIG_H
 #define __BOARD_DEFCONFIG_H
 
 #include "stm32f4xx.h"
 
 /********************************UART config**********************************************/
 /*			UART DMA Configuration      */
 //#define USARTx_USE_DMA
 
 /* 		UART1 Configuration  	 	*/
 #define CONFIG_UART1_BAUD 115200
 #define CONFIG_UART1_BITS USART_WordLength_8b
 #define CONFIG_UART1_PARITY USART_Parity_No
 #define CONFIG_UART1_2STOP USART_StopBits_1
 //#define CONFIG_UART1_DMA
 //#define CONFIG_UART1_DMA_RXBUFSIZE 300
 //#define CONFIG_UART1_DMA_TXBUFSIZE 300
 
  /* 		UART2 Configuration  	 	*/
 #define CONFIG_UART2_BAUD 115200
 #define CONFIG_UART2_BITS USART_WordLength_8b
 #define CONFIG_UART2_PARITY USART_Parity_No
 #define CONFIG_UART2_2STOP USART_StopBits_1
 //#define CONFIG_UART2_DMA
 //#define CONFIG_UART2_DMA_RXBUFSIZE 300
 //#define CONFIG_UART2_DMA_TXBUFSIZE 300
 
  /* 		UART3 Configuration  	 	*/
 #define CONFIG_UART3_BAUD 115200
 #define CONFIG_UART3_BITS USART_WordLength_8b
 #define CONFIG_UART3_PARITY USART_Parity_No
 #define CONFIG_UART3_2STOP USART_StopBits_1
 //#define CONFIG_UART3_DMA
 //#define CONFIG_UART3_DMA_RXBUFSIZE 300
 //#define CONFIG_UART3_DMA_TXBUFSIZE 300
 
  /* 		UART4 Configuration  	 	*/
 #define CONFIG_UART4_BAUD 115200
 #define CONFIG_UART4_BITS USART_WordLength_8b
 #define CONFIG_UART4_PARITY USART_Parity_No
 #define CONFIG_UART4_2STOP USART_StopBits_1
 //#define CONFIG_UART4_DMA
 //#define CONFIG_UART4_DMA_RXBUFSIZE 300
 //#define CONFIG_UART4_DMA_TXBUFSIZE 300
 
 
  /* 		UART5 Configuration  	 	*/
 #define CONFIG_UART5_BAUD 115200
 #define CONFIG_UART5_BITS USART_WordLength_8b
 #define CONFIG_UART5_PARITY USART_Parity_No
 #define CONFIG_UART5_2STOP USART_StopBits_1
 //#define CONFIG_UART5_DMA
 //#define CONFIG_UART5_DMA_RXBUFSIZE 300
 //#define CONFIG_UART5_DMA_TXBUFSIZE 300
 
  /* 		UART6 Configuration  	 	*/
 #define CONFIG_UART6_BAUD 115200
 #define CONFIG_UART6_BITS USART_WordLength_8b
 #define CONFIG_UART6_PARITY USART_Parity_No
 #define CONFIG_UART6_2STOP USART_StopBits_1
 //#define CONFIG_UART6_DMA
 //#define CONFIG_UART6_DMA_RXBUFSIZE 300
 //#define CONFIG_UART6_DMA_TXBUFSIZE 300
 
  /* 		UART7 Configuration  	 	*/
 #define CONFIG_UART7_BAUD 115200
 #define CONFIG_UART7_BITS USART_WordLength_8b
 #define CONFIG_UART7_PARITY USART_Parity_No
 #define CONFIG_UART7_2STOP USART_StopBits_1
 //#define CONFIG_UART7_DMA
 //#define CONFIG_UART7_DMA_RXBUFSIZE 300
 //#define CONFIG_UART7_DMA_TXBUFSIZE 300
 
  /* 		UART8 Configuration  	 	*/
 #define CONFIG_UART8_BAUD 115200
 #define CONFIG_UART8_BITS USART_WordLength_8b
 #define CONFIG_UART8_PARITY USART_Parity_No
 #define CONFIG_UART8_2STOP USART_StopBits_1
 //#define CONFIG_UART8_DMA
 //#define CONFIG_UART8_DMA_RXBUFSIZE 300
 //#define CONFIG_UART8_DMA_TXBUFSIZE 300
 
 /******************************************************************************/
 
 /********************************SPI config**********************************************/
	/*			spi DMA Configuration      */
 //#define SPIx_USE_DMA
 
 /* 		SPI1 Configuration  	 	*/
 #define CONFIG_STM32_HAVE_SPI1
 #define CONFIG_STM32_HAVE_SPI2
 #define CONFIG_STM32_HAVE_SPI3
 #define CONFIG_STM32_HAVE_SPI4
 #define CONFIG_STM32_HAVE_SPI5
 #define CONFIG_STM32_HAVE_SPI6
	/******************************************************************************/
	
/********************************IIC config**********************************************/
	/*			IIC DMA Configuration      */
 //#define IICx_USE_DMA
 
 /* 		IIC1 Configuration  	 	*/
 #define CONFIG_STM32_HAVE_IIC1
 #define CONFIG_STM32_HAVE_IIC2
 #define CONFIG_STM32_HAVE_IIC3
 #define CONFIG_STM32_HAVE_IIC4
/******************************************************************************/
	
/********************************WGD config**********************************************/
#define CONFIG_STM32_HAVE_WGD
/******************************************************************************/

 #endif
 
 