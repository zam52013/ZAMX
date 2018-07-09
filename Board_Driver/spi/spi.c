/****************************************************************************
 *@file spi.c
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
 #include "spi.h"
 #include "board.h"
 void SPIx_Init(SPI_Driver* SPIx)
 {
	GPIO_InitTypeDef GPIO_InitStructure;
	#ifdef SPIx_USE_DMA
	DMA_InitTypeDef DMA_InitStructure;
	#endif

	SPIx->SPI_CLK(SPIx->SPI_Func, ENABLE);
	SPIx->GPIO_CLK(SPIx->GPIO_Func, ENABLE);


	GPIO_PinAFConfig(SPIx->Gpio, SPIx->SCK_Src, SPIx->GPIO_AF_SPI);
	GPIO_PinAFConfig(SPIx->Gpio, SPIx->MISO_Src, SPIx->GPIO_AF_SPI);
	GPIO_PinAFConfig(SPIx->Gpio, SPIx->MOSI_Src, SPIx->GPIO_AF_SPI);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = SPIx->SCK_Pin | SPIx->MISO_Pin | SPIx->MOSI_Pin;
	GPIO_Init(SPIx->Gpio, &GPIO_InitStructure);

	//SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPIx->SPI);
	SPI_Init(SPIx->SPI, &SPIx->SPI_Init_Def);

	SPI_CalculateCRC(SPIx->SPI, DISABLE);

	SPI_Cmd(SPIx->SPI, ENABLE);
	while (SPI_I2S_GetFlagStatus(SPIx->SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_ReceiveData(SPIx->SPI);
	
	#ifdef SPIx_USE_DMA
	/*
	// Enable DMA clock
	SPIx->DMA_CLK(SPIx->DMA_Func, ENABLE);
	NVIC_Init(&SPIx->NVIC_DMA_TX);


	NVIC_Init(&SPIx->NVIC_DMA_RX);


	DMA_DeInit(SPIx->DMA_TX_Stream);
	while (DMA_GetCmdStatus(SPIx->DMA_TX_Stream) != DISABLE);
	DMA_Cmd(SPIx->DMA_TX_Stream, DISABLE);
	DMA_DeInit(SPIx->DMA_RX_Stream);
	while (DMA_GetCmdStatus(SPIx->DMA_RX_Stream) != DISABLE);
	DMA_Cmd(SPIx->DMA_RX_Stream, DISABLE);


	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPIx->SPI->DR));
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	// Configure TX DMA
	DMA_InitStructure.DMA_Channel = SPIx->DMA_TX_CH;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)0;
	DMA_Init(SPIx->DMA_TX_Stream, &DMA_InitStructure);
	// Configure RX DMA
	DMA_InitStructure.DMA_Channel = SPIx->DMA_RX_CH;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0; 
	DMA_Init(SPIx->DMA_RX_Stream, &DMA_InitStructure);

	SPI_DMACmd(SPIx->SPI, SPI_DMAReq_Rx, ENABLE);
	SPI_DMACmd(SPIx->SPI, SPI_DMAReq_Tx, ENABLE);

	DMA_ClearFlag(SPIx->DMA_TX_Stream, SPIx->DMA_TX_Flag);
	DMA_ClearFlag(SPIx->DMA_RX_Stream, SPIx->DMA_RX_Flag);

	DMA_ITConfig(SPIx->DMA_TX_Stream, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ITConfig(SPIx->DMA_RX_Stream, DMA_IT_TC | DMA_IT_TE, ENABLE);

	DMA_Cmd(SPIx->DMA_TX_Stream, DISABLE);
	DMA_Cmd(SPIx->DMA_RX_Stream, DISABLE);*/
	#endif
 }
 void SPIx_Init_Cs(SPIx_Cs* spi_cs)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	spi_cs->GPIO_CS_CLK(spi_cs->CS_Func, ENABLE);
	GPIO_InitStructure.GPIO_Pin = spi_cs->CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(spi_cs->GPIO_PORT_CS, &GPIO_InitStructure);
	GPIO_SetBits(spi_cs->GPIO_PORT_CS, spi_cs->CS_Pin);
}

 unsigned char SPI_ReadWriteByte(SPI_Driver* SPIx,unsigned char TxData) 
 {		 
	unsigned char retry=0;				 	 
 	while (SPI_I2S_GetFlagStatus(SPIx->SPI, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位 
 	{ 
 		retry++; 
 		if(retry>200) 
 			return 0; 
 	}			   
 	SPI_I2S_SendData(SPIx->SPI, TxData); //通过外设SPIx发送一个数据 
 	retry=0; 
 	while (SPI_I2S_GetFlagStatus(SPIx->SPI, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位 
 	{ 
 		retry++; 
 		if(retry>200) 
 			return 0; 
 	}	  						     
 	return SPI_I2S_ReceiveData(SPIx->SPI); //返回通过SPIx最近接收的数据					     
 } 
 
void stm32_spi1select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected)
{

}

 void stm32_spi2select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected)
 {
		switch(devid)
		{
			case FM25V_ID:
					if(CS_ON)
					{
						GPIO_ResetBits(spi_cs->GPIO_PORT_CS,spi_cs->CS_Pin);
					}
					else
					{
						GPIO_SetBits(spi_cs->GPIO_PORT_CS,spi_cs->CS_Pin);
					}
				break;
			default:
				break;
		}
 }


void stm32_spi3select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected)
{
	switch(devid)
		{
			case MPU9250_ID:
					if(CS_ON)
					{
						GPIO_ResetBits(spi_cs->GPIO_PORT_CS,spi_cs->CS_Pin);
					}
					else
					{
						GPIO_SetBits(spi_cs->GPIO_PORT_CS,spi_cs->CS_Pin);
					}
				break;
			default:
				break;
		}
}

void stm32_spi4select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected)
{

}

void stm32_spi5select(SPIx_Cs* spi_cs,uint32_t devid, CS_SEL selected)
{

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