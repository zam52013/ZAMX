/****************************************************************************
 *@file spi.h
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
 #ifndef __SPI_H
 #define __SPI_H
 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"
 #include "board_defconfig.h"
 
 typedef void (*RCC_AXXPeriphClockCmd)(uint32_t RCC_AXXPeriph, FunctionalState NewState);
 
 typedef struct SPI_DRIVER
{
	SPI_TypeDef* SPI;						/* SPIX device */
	RCC_AXXPeriphClockCmd SPI_CLK;		/* SPIX clock */
	uint32_t SPI_Func;					/* SPIX AHB */	
	
	GPIO_TypeDef* Gpio;					/* SPIX gpio port */			
	RCC_AXXPeriphClockCmd GPIO_CLK;		/* SPIX gpio clock */
	uint32_t GPIO_Func;					/* SPIX gpio AHB */
	
	uint16_t SCK_Pin;						/* SPIX gpio sck */
	uint16_t MISO_Pin;					/* SPIX gpio miso */
	uint16_t MOSI_Pin;					/* SPIX gpio mosi */
	uint16_t SCK_Src;						/* SPIX gpio sck src*/
	uint16_t MISO_Src;					/* SPIX gpio miso src */
	uint16_t MOSI_Src;					/* SPIX gpio mosi src */
	
	SPI_InitTypeDef SPI_Init_Def;			/* SPIX config*/

	#ifdef SPIx_USE_DMA
	RCC_AXXPeriphClockCmd DMA_CLK;
	uint32_t DMA_Func;
	DMA_TypeDef* DMA_TX;
	DMA_Stream_TypeDef* DMA_TX_Stream;
	NVIC_InitTypeDef NVIC_DMA_TX;
	uint32_t DMA_TX_CH;
	uint32_t DMA_TX_Flag;
	DMA_TypeDef* DMA_RX;
	DMA_Stream_TypeDef* DMA_RX_Stream;
	NVIC_InitTypeDef NVIC_DMA_RX;
	uint32_t DMA_RX_CH;
	uint32_t DMA_RX_Flag;
	#endif
	
	uint8_t GPIO_AF_SPI;						/* SPIX gpio af */
	
}SPI_Driver;
 
 typedef struct SPIx_CS
 {
	 GPIO_TypeDef* GPIO_PORT_CS;					/* SPIX cs gpio port */
	RCC_AXXPeriphClockCmd GPIO_CS_CLK;		/* SPIX cs gpio clock */
	uint32_t CS_Func;											/* SPIX cs gpio ahb */
	uint16_t CS_Pin;											/* SPIX cs gpio  */
}SPIx_Cs;
 
static SPI_Driver SPI_DEV1=
{
	.SPI=SPI1,
	.SPI_CLK=RCC_APB2PeriphClockCmd,
	.SPI_Func=RCC_APB2Periph_SPI1,

	.Gpio=GPIOA,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Func=RCC_AHB1Periph_GPIOA,

	.SCK_Pin=GPIO_Pin_5,
	.MISO_Pin=GPIO_Pin_6,
	.MOSI_Pin=GPIO_Pin_7,
	.SCK_Src=GPIO_PinSource5,
	.MISO_Src=GPIO_PinSource6,
	.MOSI_Src=GPIO_PinSource7,

	.SPI_Init_Def={
		.SPI_Direction=SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode=SPI_Mode_Master,
		.SPI_DataSize=SPI_DataSize_8b,
		.SPI_CPOL=SPI_CPOL_High,
		.SPI_CPHA=SPI_CPHA_2Edge,
		.SPI_NSS=SPI_NSS_Soft,
		.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4,
		.SPI_FirstBit=SPI_FirstBit_MSB,
		.SPI_CRCPolynomial=7
	},

	#ifdef SPIx_USE_DMA

	#endif
	.GPIO_AF_SPI=GPIO_AF_SPI1
};

static SPI_Driver SPI_DEV2=
{
	.SPI=SPI2,
	.SPI_CLK=RCC_APB1PeriphClockCmd,
	.SPI_Func=RCC_APB1Periph_SPI2,

	.Gpio=GPIOB,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Func=RCC_AHB1Periph_GPIOB,

	.SCK_Pin=GPIO_Pin_13,
	.MISO_Pin=GPIO_Pin_14,
	.MOSI_Pin=GPIO_Pin_15,
	.SCK_Src=GPIO_PinSource13,
	.MISO_Src=GPIO_PinSource14,
	.MOSI_Src=GPIO_PinSource15,

	.SPI_Init_Def={
		.SPI_Direction=SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode=SPI_Mode_Master,
		.SPI_DataSize=SPI_DataSize_8b,
		.SPI_CPOL=SPI_CPOL_High,
		.SPI_CPHA=SPI_CPHA_2Edge,
		.SPI_NSS=SPI_NSS_Soft,
		.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4,
		.SPI_FirstBit=SPI_FirstBit_MSB,
		.SPI_CRCPolynomial=7
	},

	#ifdef SPIx_USE_DMA

	#endif
	.GPIO_AF_SPI=GPIO_AF_SPI2
};
static SPI_Driver SPI_DEV3=
{
	.SPI=SPI3,
	.SPI_CLK=RCC_APB1PeriphClockCmd,
	.SPI_Func=RCC_APB1Periph_SPI3,

	.Gpio=GPIOC,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Func=RCC_AHB1Periph_GPIOC,

	.SCK_Pin=GPIO_Pin_10,
	.MISO_Pin=GPIO_Pin_11,
	.MOSI_Pin=GPIO_Pin_12,
	.SCK_Src=GPIO_PinSource10,
	.MISO_Src=GPIO_PinSource11,
	.MOSI_Src=GPIO_PinSource12,

	.SPI_Init_Def={
		.SPI_Direction=SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode=SPI_Mode_Master,
		.SPI_DataSize=SPI_DataSize_8b,
		.SPI_CPOL=SPI_CPOL_High,
		.SPI_CPHA=SPI_CPHA_2Edge,
		.SPI_NSS=SPI_NSS_Soft,
		.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4,
		.SPI_FirstBit=SPI_FirstBit_MSB,
		.SPI_CRCPolynomial=7
	},

	#ifdef SPIx_USE_DMA

	#endif
	.GPIO_AF_SPI=GPIO_AF_SPI3
};

static SPI_Driver SPI_DEV4=
{
	.SPI=SPI4,
	.SPI_CLK=RCC_APB2PeriphClockCmd,
	.SPI_Func=RCC_APB2Periph_SPI4,

	.Gpio=GPIOE,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Func=RCC_AHB1Periph_GPIOE,

	.SCK_Pin=GPIO_Pin_2,
	.MISO_Pin=GPIO_Pin_5,
	.MOSI_Pin=GPIO_Pin_6,
	.SCK_Src=GPIO_PinSource2,
	.MISO_Src=GPIO_PinSource5,
	.MOSI_Src=GPIO_PinSource6,

	.SPI_Init_Def={
		.SPI_Direction=SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode=SPI_Mode_Master,
		.SPI_DataSize=SPI_DataSize_8b,
		.SPI_CPOL=SPI_CPOL_High,
		.SPI_CPHA=SPI_CPHA_2Edge,
		.SPI_NSS=SPI_NSS_Soft,
		.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4,
		.SPI_FirstBit=SPI_FirstBit_MSB,
		.SPI_CRCPolynomial=7
	},

	#ifdef SPIx_USE_DMA

	#endif
	.GPIO_AF_SPI=GPIO_AF_SPI4
};


static SPI_Driver SPI_DEV5=
{
	.SPI=SPI5,
	.SPI_CLK=RCC_APB2PeriphClockCmd,
	.SPI_Func=RCC_APB2Periph_SPI5,

	.Gpio=GPIOF,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Func=RCC_AHB1Periph_GPIOF,

	.SCK_Pin=GPIO_Pin_7,
	.MISO_Pin=GPIO_Pin_8,
	.MOSI_Pin=GPIO_Pin_9,
	.SCK_Src=GPIO_PinSource7,
	.MISO_Src=GPIO_PinSource8,
	.MOSI_Src=GPIO_PinSource9,

	.SPI_Init_Def={
		.SPI_Direction=SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode=SPI_Mode_Master,
		.SPI_DataSize=SPI_DataSize_8b,
		.SPI_CPOL=SPI_CPOL_High,
		.SPI_CPHA=SPI_CPHA_2Edge,
		.SPI_NSS=SPI_NSS_Soft,
		.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4,
		.SPI_FirstBit=SPI_FirstBit_MSB,
		.SPI_CRCPolynomial=7
	},

	#ifdef SPIx_USE_DMA

	#endif
	.GPIO_AF_SPI=GPIO_AF_SPI5
};

static SPI_Driver SPI_DEV6=
{
	.SPI=SPI6,
	.SPI_CLK=RCC_APB2PeriphClockCmd,
	.SPI_Func=RCC_APB2Periph_SPI6,

	.Gpio=GPIOG,
	.GPIO_CLK=RCC_AHB1PeriphClockCmd,
	.GPIO_Func=RCC_AHB1Periph_GPIOG,

	.SCK_Pin=GPIO_Pin_13,
	.MISO_Pin=GPIO_Pin_12,
	.MOSI_Pin=GPIO_Pin_14,
	.SCK_Src=GPIO_PinSource13,
	.MISO_Src=GPIO_PinSource12,
	.MOSI_Src=GPIO_PinSource14,

	.SPI_Init_Def={
		.SPI_Direction=SPI_Direction_2Lines_FullDuplex,
		.SPI_Mode=SPI_Mode_Master,
		.SPI_DataSize=SPI_DataSize_8b,
		.SPI_CPOL=SPI_CPOL_High,
		.SPI_CPHA=SPI_CPHA_2Edge,
		.SPI_NSS=SPI_NSS_Soft,
		.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4,
		.SPI_FirstBit=SPI_FirstBit_MSB,
		.SPI_CRCPolynomial=7
	},

	#ifdef SPIx_USE_DMA

	#endif
	.GPIO_AF_SPI=GPIO_AF_SPI6
};

typedef enum
{
	CS_OFF=0,
	CS_ON
}CS_SEL;

void SPIx_Init(SPI_Driver* SPIx);
void SPIx_Init_Cs(SPIx_Cs* spi_cs);
unsigned char SPI_ReadWriteByte(SPI_Driver* SPIx,unsigned char TxData);
void stm32_spi1select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected);
void stm32_spi2select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected);
void stm32_spi3select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected);
void stm32_spi4select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected);
void stm32_spi5select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected);
 #endif
 
 
 
 