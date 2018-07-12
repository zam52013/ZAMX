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

__IO uint16_t  sEEAddress = 0;    
__IO uint16_t* sEEDataReadPointer;   
__IO uint8_t*  sEEDataWritePointer;  
__IO uint8_t   sEEDataNum;

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

///**
//  * @brief  Reads a block of data from the EEPROM.
//  * @param  pBuffer : pointer to the buffer that receives the data read from 
//  *         the EEPROM.
//  * @param  ReadAddr : EEPROM's internal address to start reading from.
//  * @param  NumByteToRead : pointer to the variable holding number of bytes to 
//  *         be read from the EEPROM.
//  * 
//  *        @note The variable pointed by NumByteToRead is reset to 0 when all the 
//  *              data are read from the EEPROM. Application should monitor this 
//  *              variable in order know when the transfer is complete.
//  * 
//  * @note When number of data to be read is higher than 1, this function just 
//  *       configures the communication and enable the DMA channel to transfer data.
//  *       Meanwhile, the user application may perform other tasks.
//  *       When number of data to be read is 1, then the DMA is not used. The byte
//  *       is read in polling mode.
//  * 
//  * @retval sEE_OK (0) if operation is correctly performed, else return value 
//  *         different from sEE_OK (0) or the timeout user callback.
//  */
//uint32_t IIC_ReadBuffer(IIC_Driver* IICx,uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead)
//{  
//  /* Set the pointer to the Number of data to be read. This pointer will be used 
//      by the DMA Transfer Completer interrupt Handler in order to reset the 
//      variable to 0. User should check on this variable in order to know if the 
//      DMA transfer has been complete or not. */
//      uint32_t sEETimeout;
//  sEEDataReadPointer = NumByteToRead;
//  
//  /*!< While the bus is busy */
//  sEETimeout = 0xfffff;
//  while(I2C_GetFlagStatus(IICx->IIC, I2C_FLAG_BUSY))
//  {
//    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
//  }
//  
//  /*!< Send START condition */
//  I2C_GenerateSTART(IICx->IIC, ENABLE);
//  
//  /*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
//  sEETimeout = 0xfffff;
//  while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_MODE_SELECT))
//  {
//    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
//  }
//  
//  /*!< Send EEPROM address for write */
//  I2C_Send7bitAddress(IICx->IIC, sEEAddress, I2C_Direction_Transmitter);

//  /*!< Test on EV6 and clear it */
//  sEETimeout = 0xfffff;
//  while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//  {
//    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
//  } 
//  
//  /*!< Send the EEPROM's internal address to read from: Only one byte address */
//  I2C_SendData(IICx->IIC, ReadAddr);  


//  /*!< Test on EV8 and clear it */
//  sEETimeout = 0xfffff;
//  while(I2C_GetFlagStatus(IICx->IIC, I2C_FLAG_BTF) == RESET)
//  {
//    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
//  }
//  
//  /*!< Send STRAT condition a second time */  
//  I2C_GenerateSTART(IICx->IIC, ENABLE);
//  
//  /*!< Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
//  sEETimeout = 0xfffff;
//  while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_MODE_SELECT))
//  {
//    if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
//  } 
//  
//  /*!< Send EEPROM address for read */
//  I2C_Send7bitAddress(IICx->IIC, sEEAddress, I2C_Direction_Receiver);  
//  
//  /* If number of data to be read is 1, then DMA couldn't be used */
//  /* One Byte Master Reception procedure (POLLING) ---------------------------*/
//  if ((uint16_t)(*NumByteToRead) < 2)
//  {
//    /* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
//    sEETimeout = 0xfffff;
//    while(I2C_GetFlagStatus(IICx->IIC, I2C_FLAG_ADDR) == RESET)
//    {
//      if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
//    }     
//    
//    /*!< Disable Acknowledgement */
//    I2C_AcknowledgeConfig(IICx->IIC, DISABLE);   
//    
//    /* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
//    (void)IICx->IIC.SR2;
//    
//    /*!< Send STOP Condition */
//    I2C_GenerateSTOP(IICx->IIC, ENABLE);
//    
//    /* Wait for the byte to be received */
//    sEETimeout = 0xfffff;
//    while(I2C_GetFlagStatus(IICx->IIC, I2C_FLAG_RXNE) == RESET)
//    {
//      if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
//    }
//    
//    /*!< Read the byte received from the EEPROM */
//    *pBuffer = I2C_ReceiveData(IICx->IIC);
//    
//    /*!< Decrement the read bytes counter */
//    (uint16_t)(*NumByteToRead)--;        
//    
//    /* Wait to make sure that STOP control bit has been cleared */
//    sEETimeout = 0xfffff;
//    while(IICx->IIC.CR1 & I2C_CR1_STOP)
//    {
//      if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
//    }  
//    
//    /*!< Re-Enable Acknowledgement to be ready for another reception */
//    I2C_AcknowledgeConfig(IICx->IIC, ENABLE);    
//  }
//  else/* More than one Byte Master Reception procedure (DMA) -----------------*/
//  {
//    /*!< Test on EV6 and clear it */
//    sEETimeout = 0xfffff;
//    while(!I2C_CheckEvent(IICx->IIC, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
//    {
//      if((sEETimeout--) == 0) return sEE_TIMEOUT_UserCallback();
//    }  
//    
//    /* Configure the DMA Rx Channel with the buffer address and the buffer size */
//    sEE_LowLevel_DMAConfig((uint32_t)pBuffer, (uint16_t)(*NumByteToRead), sEE_DIRECTION_RX);
//    
//    /* Inform the DMA that the next End Of Transfer Signal will be the last one */
//    I2C_DMALastTransferCmd(IICx->IIC, ENABLE); 
//    
//    /* Enable the DMA Rx Stream */
//    DMA_Cmd(IICx->I2C_DMA_RX, ENABLE);    

//    /* Enable the sEE_I2C peripheral DMA requests */
//    I2C_DMACmd(IICx->IIC, ENABLE);      
//  }
//  
//  /* If all operations OK, return sEE_OK (0) */
//  return 0;
//}

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
 
