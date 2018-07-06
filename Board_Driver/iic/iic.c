/****************************************************************************
 *@file iic.c
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
#include "iic.h"

void IICx_Init(IIC_Driver* IICx)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	IICx->IIC_CLK(IICx->I2C_Func,ENABLE);
	IICx->I2C_CLK_CLK(IICx->I2C_CLK_GPIOFunc,ENABLE);
	IICx->I2C_SDA_CLK(IICx->I2C_SDA_GPIOFunc,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	IICx->I2C_CLK_RESET_PER(IICx->I2C_Func,ENABLE);
	IICx->I2C_CLK_RESET_PER(IICx->I2C_Func,DISABLE);

	GPIO_PinAFConfig(IICx->I2C_CLK_PORT,IICx->I2C_CLK_Src,IICx->GPIO_AF_IICX);
	GPIO_PinAFConfig(IICx->I2C_SDA_PORT,IICx->I2C_SDA_Src,IICx->GPIO_AF_IICX);

	GPIO_InitStructure.GPIO_Pin = IICx->I2C_CLK_Pin;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(IICx->I2C_CLK_PORT,&GPIO_InitStructure);

	 GPIO_InitStructure.GPIO_Pin = IICx->I2C_SDA_Pin;
  	GPIO_Init(IICx->I2C_CLK_PORT, &GPIO_InitStructure);

	NVIC_Init(&IICx->NVIC_InitStructure_TX);
	NVIC_Init(&IICx->NVIC_InitStructure_RX);

	IICx->I2C_DMA_CLK(IICx->I2C_DMA_Func,ENABLE);
	DMA_ClearFlag(IICx->I2C_DMA_TX,IICx->DMA_FLAG_TX);

	DMA_Cmd(IICx->I2C_DMA_TX,DISABLE);
	DMA_DeInit(IICx->I2C_DMA_TX);

	DMA_Init(IICx->I2C_DMA_TX,&IICx->I2C_DMA_DEF);
	
	DMA_ClearFlag(IICx->I2C_DMA_RX,IICx->DMA_FLAG_RX);

	DMA_Cmd(IICx->I2C_DMA_RX,DISABLE);
	DMA_DeInit(IICx->I2C_DMA_RX);

	DMA_Init(IICx->I2C_DMA_RX,&IICx->I2C_DMA_DEF);

	DMA_ITConfig(IICx->I2C_DMA_TX, DMA_IT_TC, ENABLE);
 	DMA_ITConfig(IICx->I2C_DMA_RX, DMA_IT_TC, ENABLE); 

	I2C_Cmd(IICx->IIC,ENABLE);
	I2C_Init(IICx->IIC,&IICx->I2C_Init_Def);
	
}


 