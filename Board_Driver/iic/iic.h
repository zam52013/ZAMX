/****************************************************************************
 *@file iic.h
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
 #ifndef __IIC_H
 #define __IIC_H
 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"

typedef void (*RCC_AXXPeriphClockCmd)(uint32_t RCC_AXXPeriph, FunctionalState NewState);
typedef void (*RCC_AXXPeriphResetCmd)(uint32_t RCC_AXXPeriph, FunctionalState NewState);

typedef struct IIC_DRIVER
{
	I2C_TypeDef* IIC;
	RCC_AXXPeriphClockCmd IIC_CLK;
	uint32_t I2C_Func;

	GPIO_TypeDef* I2C_CLK_PORT;
	RCC_AXXPeriphClockCmd I2C_CLK_CLK;
	uint32_t I2C_CLK_GPIOFunc;
	uint32_t I2C_CLK_Pin;
	uint32_t I2C_CLK_Src;
	RCC_AXXPeriphResetCmd I2C_CLK_RESET_PER;

	GPIO_TypeDef* I2C_SDA_PORT;
	RCC_AXXPeriphClockCmd I2C_SDA_CLK;
	uint32_t I2C_SDA_GPIOFunc;
	uint32_t I2C_SDA_Pin;
	uint32_t I2C_SDA_Src;

	NVIC_InitTypeDef NVIC_InitStructure_TX;
	NVIC_InitTypeDef NVIC_InitStructure_RX;

	RCC_AXXPeriphClockCmd I2C_DMA_CLK;
	uint32_t I2C_DMA_Func;

	DMA_Stream_TypeDef* I2C_DMA_TX;
	uint32_t DMA_FLAG_TX;

	DMA_InitTypeDef I2C_DMA_DEF;

	DMA_Stream_TypeDef* I2C_DMA_RX;
	uint32_t DMA_FLAG_RX;

	I2C_InitTypeDef  I2C_Init_Def;
	
	uint8_t GPIO_AF_IICX;
}IIC_Driver;

static IIC_Driver IIC1={
	.IIC=I2C1,
	.IIC_CLK=RCC_APB1PeriphClockCmd,
	.I2C_Func=RCC_APB1Periph_I2C1,

	.I2C_CLK_PORT=GPIOB,
	.I2C_CLK_CLK=RCC_AHB1PeriphClockCmd,
	.I2C_CLK_GPIOFunc=RCC_AHB1Periph_GPIOB,
	.I2C_CLK_Pin=GPIO_Pin_6,
	.I2C_CLK_Src=GPIO_PinSource6,
	.I2C_CLK_RESET_PER=RCC_APB1PeriphResetCmd,

	.I2C_SDA_PORT=GPIOB,
	.I2C_SDA_CLK=RCC_AHB1PeriphClockCmd,
	.I2C_SDA_GPIOFunc=RCC_AHB1Periph_GPIOB,
	.I2C_SDA_Pin=GPIO_Pin_9,
	.I2C_SDA_Src=GPIO_PinSource9,

	.NVIC_InitStructure_TX=
	{
		.NVIC_IRQChannel=DMA1_Stream6_IRQn,
		.NVIC_IRQChannelPreemptionPriority=0,
		.NVIC_IRQChannelSubPriority=0,
		.NVIC_IRQChannelCmd=ENABLE
	},

	.NVIC_InitStructure_RX=
	{
		.NVIC_IRQChannel=DMA1_Stream0_IRQn,
		.NVIC_IRQChannelPreemptionPriority=0,
		.NVIC_IRQChannelSubPriority=0,
		.NVIC_IRQChannelCmd=ENABLE
	},
	.I2C_DMA_CLK=RCC_AHB1PeriphClockCmd,
	.I2C_DMA_Func=RCC_AHB1Periph_DMA1,

	.I2C_DMA_TX=DMA1_Stream6,
	.DMA_FLAG_TX=DMA_FLAG_FEIF6 |DMA_FLAG_DMEIF6  |DMA_FLAG_TEIF6 |DMA_FLAG_HTIF6 |DMA_FLAG_TCIF6,

	.I2C_DMA_DEF=
	{
		.DMA_Channel=DMA_Channel_1,
		.DMA_PeripheralBaseAddr=((uint32_t)0x40005410),
		.DMA_Memory0BaseAddr=0,
		.DMA_DIR=DMA_DIR_MemoryToPeripheral,
		.DMA_BufferSize=0xFFFF,
		.DMA_PeripheralInc=DMA_PeripheralInc_Disable,
		.DMA_MemoryInc=DMA_MemoryInc_Enable,
		.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte,
		.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte,
		.DMA_Mode=DMA_Mode_Normal,
		.DMA_Priority=DMA_Priority_VeryHigh,
		.DMA_FIFOMode=DMA_FIFOMode_Enable,
		.DMA_FIFOThreshold=DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst=DMA_MemoryBurst_Single,
		.DMA_PeripheralBurst=DMA_PeripheralBurst_Single
	},
	
	.I2C_DMA_RX=DMA1_Stream0,
	.DMA_FLAG_RX=DMA_FLAG_FEIF0 | DMA_FLAG_DMEIF0 |DMA_FLAG_TEIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_TCIF0 ,

	.I2C_Init_Def=
	{
		.I2C_ClockSpeed=100000,
		.I2C_Mode=I2C_Mode_I2C,
		.I2C_DutyCycle=I2C_DutyCycle_2,
		.I2C_OwnAddress1=0xA0,
		.I2C_Ack=I2C_Ack_Enable,
		.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit
	},
	
	.GPIO_AF_IICX=GPIO_AF_I2C1
};
	
 #endif

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
