/****************************************************************************
 *@file fm25v.h
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
 #ifndef __FM25V_H
 #define __FM25V_H
 
 #ifdef __cplusplus
 extern "C" {
	#endif
	 
	#include "board.h"
	 
	#define FM25V_WREN 0x06			/*set write enable latch*/
	#define FM25V_WRDI 0x04			/*reset write enable latch*/
	#define FM25V_RDSR 0x05			/*read status register*/
	#define FM25V_WRSR 0x01			/*write status register*/
	#define FM25V_READ 0x03			/*read memory date*/
	#define FM25V_FSTRD 0x0B		/*fast read memory date*/
	#define FM25V_WRITE 0x02		/*write memory date*/
	#define FM25V_SLEEP 0xB9		/*enter sleep mode*/
	#define FM25V_RDID 0x9f			/***?ID****/
	 
	 
	void fm25v_init(void);
	void Fram_wirte(u16 addr,u16 lenth,u8 *date);
	void Fram_read(u16 addr,u16 lenth,u8 *date);
	 
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

/******************* (C) COPYRIGHT 2018 feima *****END OF FILE****/
