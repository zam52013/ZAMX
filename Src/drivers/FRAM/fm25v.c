/****************************************************************************
 *@file fm25v.c
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
 
 #include "fm25v.h"
 
 void fm25v_init()
 {
	SPIx_Init(FM25V);
	SPIx_Init_Cs(FM25V_CS);
 }
 
  void Fram_wirte(u16 addr,u16 lenth,u8 *date)	
 {
	u8 addr_h,addr_l;
	u16 count;
	//u8 delay_cnt=10;
	addr_h=(u8)((addr&0xff00)>>8);
	addr_l=(u8)((addr&0x00ff));
	FM25V_X_select(FM25V_CS,FM25V_ID,CS_ON);
	SPI_ReadWriteByte(FM25V,FM25V_WREN);
	FM25V_X_select(FM25V_CS,FM25V_ID,CS_OFF);
	//while(delay_cnt--);
	FM25V_X_select(FM25V_CS,FM25V_ID,CS_ON);
	SPI_ReadWriteByte(FM25V,FM25V_WRITE);
	SPI_ReadWriteByte(FM25V,addr_h);
	SPI_ReadWriteByte(FM25V,addr_l);
	for(count=0;count<lenth;count++)
	{
		SPI_ReadWriteByte(FM25V,*date++);
	}
	//SPI_ReadWriteByte(FM25V_ID,FM25V_WRDI);
	FM25V_X_select(FM25V_CS,FM25V_ID,CS_OFF);
 }
 
 void Fram_read(u16 addr,u16 lenth,u8 *date)
 {
	u8 addr_h,addr_l;
	u16 count;
	addr_h=(u8)((addr&0xff00)>>8);
	addr_l=(u8)((addr&0x00ff));
	FM25V_X_select(FM25V_CS,FM25V_ID,CS_ON);
	SPI_ReadWriteByte(FM25V,FM25V_READ);
	SPI_ReadWriteByte(FM25V,addr_h);
	SPI_ReadWriteByte(FM25V,addr_l);
	for(count=0;count<lenth;count++)
	{
		*date++=SPI_ReadWriteByte(FM25V,0x00);
	}
	FM25V_X_select(FM25V_CS,FM25V_ID,CS_OFF);
 }
 
 
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
 