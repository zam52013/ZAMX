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
void IIC_DMA_Config(IIC_Driver* IICx,uint32_t pBuffer, uint32_t BufferSize, uint32_t Direction)
{
	DMA_InitTypeDef DMA_InitStructure;
	if (Direction == 0)
	{
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pBuffer;
   	 	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;    
    		DMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;  
    		DMA_Init(IICx->I2C_DMA_TX, &DMA_InitStructure);  
	}
	else
	{
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pBuffer;
   	 	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;    
    		DMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;  
    		DMA_Init(IICx->I2C_DMA_RX, &DMA_InitStructure);  
	}
}
uint16_t Read_Point_num=0;
uint32_t IICX_Read_Buffer(IIC_Driver* IICx,uint8_t Address,uint8_t ReadAddr,uint8_t* pBuffer,uint16_t Numbyte)
{
	uint32_t IIC_Out_Time;
	Read_Point_num=Numbyte;
	IIC_Out_Time=0xfff00;
	while(I2C_GetFlagStatus(IICx->IIC, I2C_FLAG_BUSY))
	{
		if((IIC_Out_Time--)==0)
			return 1;
	}
	 I2C_GenerateSTART(IICx->IIC, ENABLE);
	IIC_Out_Time=0xfff00;
	while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((IIC_Out_Time--)==0)
			return 1;
	}
	I2C_Send7bitAddress(IICx->IIC, Address, I2C_Direction_Transmitter);
	IIC_Out_Time=0xfff00;
	while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((IIC_Out_Time--)==0)
			return 1;
	}
	 I2C_SendData(IICx->IIC, ReadAddr);
	 IIC_Out_Time=0xfff00;
	 while(I2C_GetFlagStatus(IICx->IIC, I2C_FLAG_BTF) == RESET)
	{
		if((IIC_Out_Time--)==0)
			return 1;
	}
	 I2C_GenerateSTART(IICx->IIC, ENABLE);
	 IIC_Out_Time=0xfff00;
	 while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((IIC_Out_Time--)==0)
			return 1;
	}
	  I2C_Send7bitAddress(IICx->IIC, Address, I2C_Direction_Receiver); 
	  if(Numbyte<2)
	 {
		IIC_Out_Time=0xfff00;
	 	while(I2C_GetFlagStatus(IICx->IIC, I2C_FLAG_ADDR) == RESET)
		{
			if((IIC_Out_Time--)==0)
				return 1;
		}
		I2C_AcknowledgeConfig(IICx->IIC, DISABLE); 
		(void)(IICx->IIC->SR2);
		I2C_GenerateSTOP(IICx->IIC, ENABLE);
		IIC_Out_Time=0xfff00;
	 	while(I2C_GetFlagStatus(IICx->IIC, I2C_FLAG_RXNE) == RESET)
		{
			if((IIC_Out_Time--)==0)
				return 1;
		}
		*pBuffer =I2C_ReceiveData(IICx->IIC);
		Numbyte--;
		IIC_Out_Time=0xfff00;
	 	while(IICx->IIC->CR1 & I2C_CR1_STOP)
		{
			if((IIC_Out_Time--)==0)
				return 1;
		}
		I2C_AcknowledgeConfig(IICx->IIC, ENABLE);
	  }
	  else
	  {
		IIC_Out_Time=0xfff00;
	 	while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
			if((IIC_Out_Time--)==0)
				return 1;
		}
	  }
	  IIC_DMA_Config(IICx,(uint32_t)pBuffer, Numbyte,1);
	  I2C_DMALastTransferCmd(IICx->IIC, ENABLE); 
	  DMA_Cmd(IICx->I2C_DMA_RX, ENABLE);
	  return 0;
}
uint16_t Write_Point_num=0;
uint32_t IICX_Write_Buffer(IIC_Driver* IICx,uint8_t Address,uint8_t WriteAddr,uint8_t* pBuffer,uint16_t Numbyte)
{
	uint32_t IIC_Out_Time;
	Write_Point_num=Numbyte;
	IIC_Out_Time=0xfff00;
	while(I2C_GetFlagStatus(IICx->IIC, I2C_FLAG_BUSY))
	{
		if((IIC_Out_Time--)==0)
			return 1;
	}
	I2C_GenerateSTART(IICx->IIC, ENABLE);
	IIC_Out_Time=0xfff00;
	while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((IIC_Out_Time--)==0)
			return 1;
	}
	 I2C_Send7bitAddress(IICx->IIC, Address, I2C_Direction_Transmitter);
	 IIC_Out_Time=0xfff00;
	while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((IIC_Out_Time--)==0)
			return 1;
	}
	I2C_SendData(IICx->IIC, WriteAddr);
	 IIC_Out_Time=0xfff00;
	while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		if((IIC_Out_Time--)==0)
			return 1;
	}
	IIC_DMA_Config(IICx,(uint32_t)pBuffer, Numbyte,0);
	 DMA_Cmd(IICx->I2C_DMA_TX, ENABLE);
	 I2C_DMACmd(IICx->IIC, ENABLE);
  	return 0;
}

void DMA1_Stream6_IRQHandler(void)
{
	uint32_t IIC_Out_Time;
	 /* Check if the DMA transfer is complete */
 	 if(DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) != RESET)
  	{  
    	/* Disable the DMA Tx Stream and Clear TC flag */  
    		DMA_Cmd(DMA1_Stream6, DISABLE);
    		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);

    	/*!< Wait till all data have been physically transferred on the bus */
   	 IIC_Out_Time =0xfff00 ;
    	while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF))
    	{
      		if((IIC_Out_Time--)==0)
			return ;
    	}
    
    	/*!< Send STOP condition */
    	I2C_GenerateSTOP(I2C1, ENABLE);
    
    	/* Reset the variable holding the number of data to be written */
    	Write_Point_num = 0;  
  }
}
void DMA1_Stream0_IRQHandler(void)
{
	 /* Check if the DMA transfer is complete */
  	if(DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) != RESET)
  	{      
   		 /*!< Send STOP Condition */
   	 	I2C_GenerateSTOP(I2C1, ENABLE);    
    
   		 /* Disable the DMA Rx Stream and Clear TC Flag */  
    		DMA_Cmd(DMA1_Stream0, DISABLE);
    		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
    
    		/* Reset the variable holding the number of data to be read */
    		Read_Point_num = 0;
  	}
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
 
