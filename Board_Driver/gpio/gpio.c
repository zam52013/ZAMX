/****************************************************************************
 *@file gpio.c
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
 #include "gpio.h"
 
 void GPIOx_Init(GPIO_Driver* GPIOx)
 {
	GPIOx->GPIO_CLK(GPIOx->GPIO_Func,ENABLE);
	GPIO_Init(GPIOx->GPIO_X,&GPIOx->GPIO_Init_Def);
 }
 
 void GPIOx_Write(GPIO_Driver* GPIOx,GPIO_State fun_state)
{

	switch(fun_state)
	{
		case ON:
			GPIO_SetBits(GPIOx->GPIO_X,GPIOx->GPIO_Init_Def.GPIO_Pin);
			break;
			
		case OFF:
			GPIO_ResetBits(GPIOx->GPIO_X,GPIOx->GPIO_Init_Def.GPIO_Pin);
			break;
			
		case ON_OFF:
			GPIO_WriteBit(GPIOx->GPIO_X,GPIOx->GPIO_Init_Def.GPIO_Pin,!GPIO_ReadOutputDataBit(GPIOx->GPIO_X,GPIOx->GPIO_Init_Def.GPIO_Pin));
			break;
			
		default:
			break;
	}

 }

uint8_t GPIOx_Read_Input(GPIO_Driver* GPIOx)
{
	return GPIO_ReadInputDataBit(GPIOx->GPIO_X,GPIOx->GPIO_Init_Def.GPIO_Pin);
}

uint8_t GPIOx_Read_Out(GPIO_Driver* GPIOx)
{
	return GPIO_ReadOutputDataBit(GPIOx->GPIO_X,GPIOx->GPIO_Init_Def.GPIO_Pin);
}
 
	/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT feima *****END OF FILE****/