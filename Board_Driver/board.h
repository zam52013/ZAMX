/****************************************************************************
 *@file board.h
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
 * @date 2018-07-05
 * 
 * 
 ****************************************************************************/
 #ifndef __BOARD_H
 #define __BOARD_H
 
  #ifdef __cplusplus
 extern "C" {
	#endif
	 
	#include "stm32f4xx.h"
 #include "iic.h"
 #include "spi.h"
 #include "times.h"
 #include "uart.h"
 #include "gpio.h"
 
/********************************PERPH DEVICE config**********************************************/

/***fram***/
static SPIx_Cs FMXV_CS=
{
		.GPIO_PORT_CS=GPIOA,
		.GPIO_CS_CLK=RCC_AHB1PeriphClockCmd,
		.CS_Func=RCC_AHB1Periph_GPIOA,
		.CS_Pin=GPIO_Pin_5
};

#define FM25V_ID 1
#define FM25V_X_select	stm32_spi2select
static SPI_Driver* FM25V=&SPI_DEV2;	 
static SPIx_Cs* FM25V_CS=&FMXV_CS;

/***key***/
static GPIO_Driver KEY_Gpio=
{
	.GPIO_X=GPIOA,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Init_Def=
	{
		.GPIO_Pin=GPIO_Pin_7,
		.GPIO_Mode=GPIO_Mode_IN,
		.GPIO_Speed=GPIO_Speed_50MHz,
		.GPIO_OType=GPIO_OType_PP,
		.GPIO_PuPd=GPIO_PuPd_UP
	},
	.GPIO_Func=RCC_AHB1Periph_GPIOA
};
static GPIO_Driver* KEY_GPIO=&KEY_Gpio;

/***MPU9250***/
static SPIx_Cs MPUX_CS=
{
		.GPIO_PORT_CS=GPIOA,
		.GPIO_CS_CLK=RCC_AHB1PeriphClockCmd,
		.CS_Func=RCC_AHB1Periph_GPIOA,
		.CS_Pin=GPIO_Pin_5
};

#define MPU9250_ID 2
#define MPU9250_X_select	stm32_spi3select
static SPI_Driver* MPU9250=&SPI_DEV3;	 
static SPIx_Cs* MPU9250_CS=&MPUX_CS;


/***LED***/

static GPIO_Driver POW_Gpio1=
{
	.GPIO_X=GPIOF,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Init_Def=
	{
		.GPIO_Pin=GPIO_Pin_8,
		.GPIO_Mode=GPIO_Mode_OUT,
		.GPIO_Speed=GPIO_Speed_100MHz,
		.GPIO_OType=GPIO_OType_PP,
		.GPIO_PuPd=GPIO_PuPd_NOPULL
	},
	.GPIO_Func=RCC_AHB1Periph_GPIOF
};
static GPIO_Driver* POW_GPIO1=&POW_Gpio1;

static GPIO_Driver POW_Gpio2=
{
	.GPIO_X=GPIOF,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Init_Def=
	{
		.GPIO_Pin=GPIO_Pin_9,
		.GPIO_Mode=GPIO_Mode_OUT,
		.GPIO_Speed=GPIO_Speed_100MHz,
		.GPIO_OType=GPIO_OType_PP,
		.GPIO_PuPd=GPIO_PuPd_NOPULL
	},
	.GPIO_Func=RCC_AHB1Periph_GPIOF
};
static GPIO_Driver* POW_GPIO2=&POW_Gpio2;

static GPIO_Driver STAT_Gpio1=
{
	.GPIO_X=GPIOF,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Init_Def=
	{
		.GPIO_Pin=GPIO_Pin_10,
		.GPIO_Mode=GPIO_Mode_OUT,
		.GPIO_Speed=GPIO_Speed_100MHz,
		.GPIO_OType=GPIO_OType_PP,
		.GPIO_PuPd=GPIO_PuPd_NOPULL
	},
	.GPIO_Func=RCC_AHB1Periph_GPIOF
};
static GPIO_Driver* STAT_GPIO1=&STAT_Gpio1;

static GPIO_Driver STAT_Gpio2=
{
	.GPIO_X=GPIOF,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Init_Def=
	{
		.GPIO_Pin=GPIO_Pin_11,
		.GPIO_Mode=GPIO_Mode_OUT,
		.GPIO_Speed=GPIO_Speed_100MHz,
		.GPIO_OType=GPIO_OType_PP,
		.GPIO_PuPd=GPIO_PuPd_NOPULL
	},
	.GPIO_Func=RCC_AHB1Periph_GPIOF
};
static GPIO_Driver* STAT_GPIO2=&STAT_Gpio2;

/***POWER CONTRO***/
static GPIO_Driver POW_Gpio=
{
	.GPIO_X=GPIOF,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Init_Def=
	{
		.GPIO_Pin=GPIO_Pin_12,
		.GPIO_Mode=GPIO_Mode_OUT,
		.GPIO_Speed=GPIO_Speed_100MHz,
		.GPIO_OType=GPIO_OType_PP,
		.GPIO_PuPd=GPIO_PuPd_NOPULL
	},
	.GPIO_Func=RCC_AHB1Periph_GPIOF
};
static GPIO_Driver* POW_GPIO=&POW_Gpio;

/***SOUCE init***/
static USART_Driver* SOUCE_PER=&Seril_A;

/***SIM7600 init***/
static USART_Driver* SIM7600_PER=&Seril_A;
#define SIM7600_IRTHandler USART1_IRQHandler
#define GSM_USART USART1

static GPIO_Driver SIM7600_POW_Gpio=
{
	.GPIO_X=GPIOF,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Init_Def=
	{
		.GPIO_Pin=GPIO_Pin_12,
		.GPIO_Mode=GPIO_Mode_OUT,
		.GPIO_Speed=GPIO_Speed_100MHz,
		.GPIO_OType=GPIO_OType_PP,
		.GPIO_PuPd=GPIO_PuPd_NOPULL
	},
	.GPIO_Func=RCC_AHB1Periph_GPIOF
};
static GPIO_Driver* SIM7600_POW_GPIO=&SIM7600_POW_Gpio;

static GPIO_Driver SIM7600_FLIGHT_Gpio=
{
	.GPIO_X=GPIOF,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Init_Def=
	{
		.GPIO_Pin=GPIO_Pin_12,
		.GPIO_Mode=GPIO_Mode_OUT,
		.GPIO_Speed=GPIO_Speed_100MHz,
		.GPIO_OType=GPIO_OType_PP,
		.GPIO_PuPd=GPIO_PuPd_NOPULL
	},
	.GPIO_Func=RCC_AHB1Periph_GPIOF
};
static GPIO_Driver* SIM7600_FLIGHT_GPIO=&SIM7600_FLIGHT_Gpio;


/***RTK init***/
static USART_Driver* RTK_DAT=&Seril_A;
static USART_Driver* RTK_RTCM=&Seril_A;

/************************************************************************************************/


#ifdef __cplusplus
}
#endif
 
 #endif
 
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
 
 