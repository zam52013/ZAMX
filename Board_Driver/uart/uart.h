/****************************************************************************
 *@file uart.h
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
 
 #ifndef __UART_H
 #define __UART_H
 
 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"
 #include "board_defconfig.h"
 
typedef void (*RCC_AXXPeriphClockCmd)(uint32_t RCC_AXXPeriph, FunctionalState NewState);
 
 typedef struct UART_DRIVER
 {
	USART_TypeDef* USART;					/* USART_x device */
	RCC_AXXPeriphClockCmd USART_CLK;		/* USART_x device clock*/
	uint32_t USART_Func;						/* USART_x device AHB*/
	
	USART_InitTypeDef InitTypeDef;				/* USART_x device config */
	
	GPIO_TypeDef* TX_GPIO;					/* USART_x device TX port*/
	RCC_AXXPeriphClockCmd TX_GPIOClk;		/* USART_x device TX port clock*/
	uint32_t TX_GPIOFunc;					/* USART_x device TX port clock AHB*/
	uint16_t TX_Pin;							/* USART_x device TX port io*/
	uint16_t TX_Src;							/* USART_x device TX port io src*/
		
	GPIO_TypeDef* RX_GPIO;					/* USART_x device RX port*/
	RCC_AXXPeriphClockCmd RX_GPIOClk;		/* USART_x device RX port clock*/
	uint32_t RX_GPIOFunc;					/* USART_x device RX port clock AHB*/
	uint16_t RX_Pin;							/* USART_x device RX port io*/
	uint16_t RX_Src;							/* USART_x device TX port io src*/

	NVIC_InitTypeDef NVIC_USART;				/* USART_x device NVIC Config*/
	uint16_t NVIC_FLAG;						/* USART_x device NVIC flag Config*/
	uint8_t USE_DMA;							/* if true use dma*/
	
	#ifdef USARTx_USE_DMA
	NVIC_InitTypeDef NVIC_DMA_TX;			/* USART_x device DMA TX NVIC*/
	
	RCC_AXXPeriphClockCmd DMA_CLK;			/* USART_x device DMA clock*/
	uint32_t DMA_Func;						/* USART_x device DMA clock AHB*/
	
	uint32_t DMA_TX_Size;					/* USART_x device tx DMA size*/
	int8_t* DMA_TX_Buffer;					/* USART_x device tx DMA buffer*/
	DMA_Stream_TypeDef* DMA_TX_Stream;		/* USART_x device DMA tx stream*/
	uint32_t DMA_TX_CH;						/* USART_x device DMA tx channel*/
	uint16_t NVIC_TI_FLAG;					/* USART_x device DMA tx ti flag*/
	
	uint32_t DMA_RX_Size;					/* USART_x device rx DMA size*/
	int8_t* DMA_RX_Buffer;					/* USART_x device rx DMA buffer*/
	DMA_Stream_TypeDef* DMA_RX_Stream;		/* USART_x device DMA rx stream*/
	uint32_t DMA_RX_CH;						/* USART_x device DMA rx channel*/
	#endif

	uint8_t GPIO_AF_USART;					/* USART_x device AF*/
 }USART_Driver;
 
#ifdef CONFIG_UART1_DMA
static int8_t g_usart1txfifo[CONFIG_UART1_DMA_TXBUFSIZE];
static int8_t g_usart1rxfifo[CONFIG_UART1_DMA_RXBUFSIZE];
#endif

 static USART_Driver Seril_A=
 {
 	.USART=USART1,
	.USART_CLK=RCC_APB2PeriphClockCmd,
	.USART_Func=RCC_APB2Periph_USART1,
	
 	.InitTypeDef={
 		.USART_BaudRate=CONFIG_UART1_BAUD,
		.USART_WordLength=CONFIG_UART1_BITS,
		.USART_StopBits=CONFIG_UART1_2STOP,
		.USART_Parity=CONFIG_UART1_PARITY,
		.USART_Mode=USART_Mode_Rx | USART_Mode_Tx,
		.USART_HardwareFlowControl=USART_HardwareFlowControl_None
		},
		
 	.TX_GPIO=GPIOA,
 	.TX_GPIOClk=RCC_AHB1PeriphClockCmd,
 	.TX_GPIOFunc=RCC_AHB1Periph_GPIOA,
 	.TX_Pin=GPIO_Pin_9,
 	.TX_Src=GPIO_PinSource9,
 	
 	.RX_GPIO=GPIOA,
 	.RX_GPIOClk=RCC_AHB1PeriphClockCmd , 
 	.RX_GPIOFunc=RCC_AHB1Periph_GPIOA, 
 	.RX_Pin=GPIO_Pin_10, 
 	.RX_Src=GPIO_PinSource10,
 	
 	.NVIC_USART={ 
 		.NVIC_IRQChannel=USART1_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1,
		.NVIC_IRQChannelSubPriority=2,
		.NVIC_IRQChannelCmd=ENABLE 
	},
	
 	.NVIC_FLAG=USART_IT_IDLE | USART_IT_RXNE,
 	
 	#ifdef CONFIG_UART1_DMA
	.USE_DMA=1,
 	.NVIC_DMA_TX={
 		.NVIC_IRQChannel=DMA2_Stream7_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1, 
		.NVIC_IRQChannelSubPriority=3, 
		.NVIC_IRQChannelCmd=ENABLE 
	},
 	.DMA_CLK=RCC_AHB1PeriphClockCmd, 
 	.DMA_Func=RCC_AHB1Periph_DMA2,
 	
 	.DMA_TX_Size=CONFIG_UART1_DMA_TXBUFSIZE, 
 	.DMA_TX_Buffer=g_usart1txfifo, 
 	.DMA_TX_Stream=DMA2_Stream7, 
 	.DMA_TX_CH=DMA_Channel_4,
 	.NVIC_TI_FLAG=DMA_IT_TC,
 	
	.DMA_RX_Size=CONFIG_UART1_DMA_RXBUFSIZE,
	.DMA_RX_Buffer=g_usart1rxfifo, 
	.DMA_RX_Stream=DMA2_Stream5, 
	.DMA_RX_CH=DMA_Channel_4,
	#else
	.USE_DMA=0,
	#endif
	
	.GPIO_AF_USART=GPIO_AF_USART1
};

#ifdef CONFIG_UART2_DMA
static int8_t g_usart2txfifo[CONFIG_UART2_DMA_TXBUFSIZE];
static int8_t g_usart2rxfifo[CONFIG_UART2_DMA_RXBUFSIZE];
#endif

 static USART_Driver Seril_B=
 {
 	.USART=USART2,
	.USART_CLK=RCC_APB1PeriphClockCmd,
	.USART_Func=RCC_APB1Periph_USART2,
	
 	.InitTypeDef={
 		.USART_BaudRate=CONFIG_UART2_BAUD,
		.USART_WordLength=CONFIG_UART2_BITS,
		.USART_StopBits=CONFIG_UART2_2STOP,
		.USART_Parity=CONFIG_UART2_PARITY,
		.USART_Mode=USART_Mode_Rx | USART_Mode_Tx,
		.USART_HardwareFlowControl=USART_HardwareFlowControl_None
		},
		
 	.TX_GPIO=GPIOD,
 	.TX_GPIOClk=RCC_AHB1PeriphClockCmd,
 	.TX_GPIOFunc=RCC_AHB1Periph_GPIOD,
 	.TX_Pin=GPIO_Pin_5,
 	.TX_Src=GPIO_PinSource5,
 	
 	.RX_GPIO=GPIOD,
 	.RX_GPIOClk=RCC_AHB1PeriphClockCmd , 
 	.RX_GPIOFunc=RCC_AHB1Periph_GPIOD, 
 	.RX_Pin=GPIO_Pin_6, 
 	.RX_Src=GPIO_PinSource6,
 	
 	.NVIC_USART={ 
 		.NVIC_IRQChannel=USART2_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1,
		.NVIC_IRQChannelSubPriority=2,
		.NVIC_IRQChannelCmd=ENABLE 
	},
	
 	.NVIC_FLAG=USART_IT_IDLE | USART_IT_RXNE,
 	
 	#ifdef CONFIG_UART2_DMA
	.USE_DMA=1,
 	.NVIC_DMA_TX={
 		.NVIC_IRQChannel=DMA2_Stream7_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1, 
		.NVIC_IRQChannelSubPriority=3, 
		.NVIC_IRQChannelCmd=ENABLE 
	},
 	.DMA_CLK=RCC_AHB1PeriphClockCmd, 
 	.DMA_Func=RCC_AHB1Periph_DMA2,
 	
 	.DMA_TX_Size=CONFIG_UART2_DMA_TXBUFSIZE, 
 	.DMA_TX_Buffer=g_usart2txfifo, 
 	.DMA_TX_Stream=DMA2_Stream7, 
 	.DMA_TX_CH=DMA_Channel_4,
 	.NVIC_TI_FLAG=DMA_IT_TC,
 	
	.DMA_RX_Size=CONFIG_UART2_DMA_RXBUFSIZE,
	.DMA_RX_Buffer=g_usart2rxfifo, 
	.DMA_RX_Stream=DMA2_Stream5, 
	.DMA_RX_CH=DMA_Channel_4,
	#else
	.USE_DMA=0,
	#endif
	
	.GPIO_AF_USART=GPIO_AF_USART2
};

#ifdef CONFIG_UART3_DMA
static int8_t g_usart3txfifo[CONFIG_UART3_DMA_TXBUFSIZE];
static int8_t g_usart3rxfifo[CONFIG_UART3_DMA_RXBUFSIZE];
#endif

 static USART_Driver Seril_C=
 {
 	.USART=USART3,
	.USART_CLK=RCC_APB1PeriphClockCmd,
	.USART_Func=RCC_APB1Periph_USART3,
	
 	.InitTypeDef={
 		.USART_BaudRate=CONFIG_UART3_BAUD,
		.USART_WordLength=CONFIG_UART3_BITS,
		.USART_StopBits=CONFIG_UART3_2STOP,
		.USART_Parity=CONFIG_UART3_PARITY,
		.USART_Mode=USART_Mode_Rx | USART_Mode_Tx,
		.USART_HardwareFlowControl=USART_HardwareFlowControl_None
		},
		
 	.TX_GPIO=GPIOD,
 	.TX_GPIOClk=RCC_AHB1PeriphClockCmd,
 	.TX_GPIOFunc=RCC_AHB1Periph_GPIOD,
 	.TX_Pin=GPIO_Pin_8,
 	.TX_Src=GPIO_PinSource8,
 	
 	.RX_GPIO=GPIOD,
 	.RX_GPIOClk=RCC_AHB1PeriphClockCmd , 
 	.RX_GPIOFunc=RCC_AHB1Periph_GPIOD, 
 	.RX_Pin=GPIO_Pin_9, 
 	.RX_Src=GPIO_PinSource9,
 	
 	.NVIC_USART={ 
 		.NVIC_IRQChannel=USART3_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1,
		.NVIC_IRQChannelSubPriority=2,
		.NVIC_IRQChannelCmd=ENABLE 
	},
	
 	.NVIC_FLAG=USART_IT_IDLE | USART_IT_RXNE,
 	
 	#ifdef CONFIG_UART3_DMA
	.USE_DMA=1,
 	.NVIC_DMA_TX={
 		.NVIC_IRQChannel=DMA2_Stream7_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1, 
		.NVIC_IRQChannelSubPriority=3, 
		.NVIC_IRQChannelCmd=ENABLE 
	},
 	.DMA_CLK=RCC_AHB1PeriphClockCmd, 
 	.DMA_Func=RCC_AHB1Periph_DMA2,
 	
 	.DMA_TX_Size=CONFIG_UART3_DMA_TXBUFSIZE, 
 	.DMA_TX_Buffer=g_usart3txfifo, 
 	.DMA_TX_Stream=DMA2_Stream7, 
 	.DMA_TX_CH=DMA_Channel_4,
 	.NVIC_TI_FLAG=DMA_IT_TC,
 	
	.DMA_RX_Size=CONFIG_UART3_DMA_RXBUFSIZE,
	.DMA_RX_Buffer=g_usart3rxfifo, 
	.DMA_RX_Stream=DMA2_Stream5, 
	.DMA_RX_CH=DMA_Channel_4,
	#else
	.USE_DMA=0,
	#endif
	
	.GPIO_AF_USART=GPIO_AF_USART3
};


#ifdef CONFIG_UART4_DMA
static int8_t g_usart4txfifo[CONFIG_UART4_DMA_TXBUFSIZE];
static int8_t g_usart4rxfifo[CONFIG_UART4_DMA_RXBUFSIZE];
#endif

 static USART_Driver Seril_D=
 {
 	.USART=UART4,
	.USART_CLK=RCC_APB1PeriphClockCmd,
	.USART_Func=RCC_APB1Periph_UART4,
	
 	.InitTypeDef={
 		.USART_BaudRate=CONFIG_UART4_BAUD,
		.USART_WordLength=CONFIG_UART4_BITS,
		.USART_StopBits=CONFIG_UART4_2STOP,
		.USART_Parity=CONFIG_UART4_PARITY,
		.USART_Mode=USART_Mode_Rx | USART_Mode_Tx,
		.USART_HardwareFlowControl=USART_HardwareFlowControl_None
		},
		
 	.TX_GPIO=GPIOA,
 	.TX_GPIOClk=RCC_AHB1PeriphClockCmd,
 	.TX_GPIOFunc=RCC_AHB1Periph_GPIOA,
 	.TX_Pin=GPIO_Pin_0,
 	.TX_Src=GPIO_PinSource0,
 	
 	.RX_GPIO=GPIOA,
 	.RX_GPIOClk=RCC_AHB1PeriphClockCmd , 
 	.RX_GPIOFunc=RCC_AHB1Periph_GPIOA, 
 	.RX_Pin=GPIO_Pin_1, 
 	.RX_Src=GPIO_PinSource1,
 	
 	.NVIC_USART={ 
 		.NVIC_IRQChannel=UART4_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1,
		.NVIC_IRQChannelSubPriority=2,
		.NVIC_IRQChannelCmd=ENABLE 
	},
	
 	.NVIC_FLAG=USART_IT_IDLE | USART_IT_RXNE,
 	
 	#ifdef CONFIG_UART4_DMA
	.USE_DMA=1,
 	.NVIC_DMA_TX={
 		.NVIC_IRQChannel=DMA2_Stream7_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1, 
		.NVIC_IRQChannelSubPriority=3, 
		.NVIC_IRQChannelCmd=ENABLE 
	},
 	.DMA_CLK=RCC_AHB1PeriphClockCmd, 
 	.DMA_Func=RCC_AHB1Periph_DMA2,
 	
 	.DMA_TX_Size=CONFIG_UART4_DMA_TXBUFSIZE, 
 	.DMA_TX_Buffer=g_usart4txfifo, 
 	.DMA_TX_Stream=DMA2_Stream7, 
 	.DMA_TX_CH=DMA_Channel_4,
 	.NVIC_TI_FLAG=DMA_IT_TC,
 	
	.DMA_RX_Size=CONFIG_UART4_DMA_RXBUFSIZE,
	.DMA_RX_Buffer=g_usart4rxfifo, 
	.DMA_RX_Stream=DMA2_Stream5, 
	.DMA_RX_CH=DMA_Channel_4,
	#else
	.USE_DMA=0,
	#endif
	
	.GPIO_AF_USART=GPIO_AF_UART4
};

#ifdef CONFIG_UART5_DMA
static int8_t g_usart5txfifo[CONFIG_UART5_DMA_TXBUFSIZE];
static int8_t g_usart5rxfifo[CONFIG_UART5_DMA_RXBUFSIZE];
#endif

 static USART_Driver Seril_E=
 {
 	.USART=UART5,
	.USART_CLK=RCC_APB1PeriphClockCmd,
	.USART_Func=RCC_APB1Periph_UART5,
	
 	.InitTypeDef={
 		.USART_BaudRate=CONFIG_UART5_BAUD,
		.USART_WordLength=CONFIG_UART5_BITS,
		.USART_StopBits=CONFIG_UART5_2STOP,
		.USART_Parity=CONFIG_UART5_PARITY,
		.USART_Mode=USART_Mode_Rx | USART_Mode_Tx,
		.USART_HardwareFlowControl=USART_HardwareFlowControl_None
		},
		
 	.TX_GPIO=GPIOC,
 	.TX_GPIOClk=RCC_AHB1PeriphClockCmd,
 	.TX_GPIOFunc=RCC_AHB1Periph_GPIOC,
 	.TX_Pin=GPIO_Pin_12,
 	.TX_Src=GPIO_PinSource12,
 	
 	.RX_GPIO=GPIOD,
 	.RX_GPIOClk=RCC_AHB1PeriphClockCmd , 
 	.RX_GPIOFunc=RCC_AHB1Periph_GPIOD, 
 	.RX_Pin=GPIO_Pin_2, 
 	.RX_Src=GPIO_PinSource2,
 	
 	.NVIC_USART={ 
 		.NVIC_IRQChannel=UART5_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1,
		.NVIC_IRQChannelSubPriority=2,
		.NVIC_IRQChannelCmd=ENABLE 
	},
	
 	.NVIC_FLAG=USART_IT_IDLE | USART_IT_RXNE,
 	
 	#ifdef CONFIG_UART5_DMA
	.USE_DMA=1,
 	.NVIC_DMA_TX={
 		.NVIC_IRQChannel=DMA2_Stream7_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1, 
		.NVIC_IRQChannelSubPriority=3, 
		.NVIC_IRQChannelCmd=ENABLE 
	},
 	.DMA_CLK=RCC_AHB1PeriphClockCmd, 
 	.DMA_Func=RCC_AHB1Periph_DMA2,
 	
 	.DMA_TX_Size=CONFIG_UART5_DMA_TXBUFSIZE, 
 	.DMA_TX_Buffer=g_usart5txfifo, 
 	.DMA_TX_Stream=DMA2_Stream7, 
 	.DMA_TX_CH=DMA_Channel_4,
 	.NVIC_TI_FLAG=DMA_IT_TC,
 	
	.DMA_RX_Size=CONFIG_UART5_DMA_RXBUFSIZE,
	.DMA_RX_Buffer=g_usart5rxfifo, 
	.DMA_RX_Stream=DMA2_Stream5, 
	.DMA_RX_CH=DMA_Channel_4,
	#else
	.USE_DMA=0,
	#endif
	
	.GPIO_AF_USART=GPIO_AF_UART5
};

#ifdef CONFIG_UART6_DMA
static int8_t g_usart6txfifo[CONFIG_UART6_DMA_TXBUFSIZE];
static int8_t g_usart6rxfifo[CONFIG_UART6_DMA_RXBUFSIZE];
#endif

 static USART_Driver Seril_F=
 {
 	.USART=USART6,
	.USART_CLK=RCC_APB2PeriphClockCmd,
	.USART_Func=RCC_APB2Periph_USART6,
	
 	.InitTypeDef={
 		.USART_BaudRate=CONFIG_UART6_BAUD,
		.USART_WordLength=CONFIG_UART6_BITS,
		.USART_StopBits=CONFIG_UART6_2STOP,
		.USART_Parity=CONFIG_UART6_PARITY,
		.USART_Mode=USART_Mode_Rx | USART_Mode_Tx,
		.USART_HardwareFlowControl=USART_HardwareFlowControl_None
		},
		
 	.TX_GPIO=GPIOG,
 	.TX_GPIOClk=RCC_AHB1PeriphClockCmd,
 	.TX_GPIOFunc=RCC_AHB1Periph_GPIOG,
 	.TX_Pin=GPIO_Pin_14,
 	.TX_Src=GPIO_PinSource14,
 	
 	.RX_GPIO=GPIOG,
 	.RX_GPIOClk=RCC_AHB1PeriphClockCmd , 
 	.RX_GPIOFunc=RCC_AHB1Periph_GPIOG, 
 	.RX_Pin=GPIO_Pin_9, 
 	.RX_Src=GPIO_PinSource9,
 	
 	.NVIC_USART={ 
 		.NVIC_IRQChannel=USART6_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1,
		.NVIC_IRQChannelSubPriority=2,
		.NVIC_IRQChannelCmd=ENABLE 
	},
	
 	.NVIC_FLAG=USART_IT_IDLE | USART_IT_RXNE,
 	
 	#ifdef CONFIG_UART6_DMA
	.USE_DMA=1,
 	.NVIC_DMA_TX={
 		.NVIC_IRQChannel=DMA2_Stream7_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1, 
		.NVIC_IRQChannelSubPriority=3, 
		.NVIC_IRQChannelCmd=ENABLE 
	},
 	.DMA_CLK=RCC_AHB1PeriphClockCmd, 
 	.DMA_Func=RCC_AHB1Periph_DMA2,
 	
 	.DMA_TX_Size=CONFIG_UART6_DMA_TXBUFSIZE, 
 	.DMA_TX_Buffer=g_usart6txfifo, 
 	.DMA_TX_Stream=DMA2_Stream7, 
 	.DMA_TX_CH=DMA_Channel_4,
 	.NVIC_TI_FLAG=DMA_IT_TC,
 	
	.DMA_RX_Size=CONFIG_UART6_DMA_RXBUFSIZE,
	.DMA_RX_Buffer=g_usart6rxfifo, 
	.DMA_RX_Stream=DMA2_Stream5, 
	.DMA_RX_CH=DMA_Channel_4,
	#else
	.USE_DMA=0,
	#endif
	
	.GPIO_AF_USART=GPIO_AF_USART6
};
/*
#ifdef CONFIG_UART7_DMA
static int8_t g_usart7txfifo[CONFIG_UART7_DMA_TXBUFSIZE];
static int8_t g_usart7rxfifo[CONFIG_UART7_DMA_RXBUFSIZE];
#endif

 static USART_Driver Seril_G=
 {
 	.USART=UART7,
	.USART_CLK=RCC_APB1PeriphClockCmd,
	.USART_Func=RCC_APB1Periph_UART7,
	
 	.InitTypeDef={
 		.USART_BaudRate=CONFIG_UART7_BAUD,
		.USART_WordLength=CONFIG_UART7_BITS,
		.USART_StopBits=CONFIG_UART7_2STOP,
		.USART_Parity=CONFIG_UART7_PARITY,
		.USART_Mode=USART_Mode_Rx | USART_Mode_Tx,
		.USART_HardwareFlowControl=USART_HardwareFlowControl_None
		},
		
 	.TX_GPIO=GPIOE,
 	.TX_GPIOClk=RCC_AHB1PeriphClockCmd,
 	.TX_GPIOFunc=RCC_AHB1Periph_GPIOE,
 	.TX_Pin=GPIO_Pin_8,
 	.TX_Src=GPIO_PinSource8,
 	
 	.RX_GPIO=GPIOE,
 	.RX_GPIOClk=RCC_AHB1PeriphClockCmd , 
 	.RX_GPIOFunc=RCC_AHB1Periph_GPIOE, 
 	.RX_Pin=GPIO_Pin_7, 
 	.RX_Src=GPIO_PinSource7,
 	
 	.NVIC_USART={ 
 		.NVIC_IRQChannel=UART7_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1,
		.NVIC_IRQChannelSubPriority=2,
		.NVIC_IRQChannelCmd=ENABLE 
	},
	
 	.NVIC_FLAG=USART_IT_IDLE | USART_IT_RXNE,
 	
 	#ifdef CONFIG_UART7_DMA
	.USE_DMA=1,
 	.NVIC_DMA_TX={
 		.NVIC_IRQChannel=DMA2_Stream7_IRQn, 
		.NVIC_IRQChannelPreemptionPriority=1, 
		.NVIC_IRQChannelSubPriority=3, 
		.NVIC_IRQChannelCmd=ENABLE 
	},
 	.DMA_CLK=RCC_AHB1PeriphClockCmd, 
 	.DMA_Func=RCC_AHB1Periph_DMA2,
 	
 	.DMA_TX_Size=CONFIG_UART7_DMA_TXBUFSIZE, 
 	.DMA_TX_Buffer=g_usart7txfifo, 
 	.DMA_TX_Stream=DMA2_Stream7, 
 	.DMA_TX_CH=DMA_Channel_4,
 	.NVIC_TI_FLAG=DMA_IT_TC,
 	
	.DMA_RX_Size=CONFIG_UART7_DMA_RXBUFSIZE,
	.DMA_RX_Buffer=g_usart7rxfifo, 
	.DMA_RX_Stream=DMA2_Stream5, 
	.DMA_RX_CH=DMA_Channel_4,
	#else
	.USE_DMA=0,
	#endif
	
	.GPIO_AF_USART=GPIO_AF_UART7
};
*/
  void UARTx_Init(USART_Driver* USARTx);
  void USARTx_DeInit(USART_Driver* USARTx);
  void USARTx_SendBytes(USART_Driver* USARTx, uint8_t* buffer, uint8_t length);
 #endif
 
 
 
 
 
 
 
