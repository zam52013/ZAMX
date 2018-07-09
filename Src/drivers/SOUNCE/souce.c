/****************************************************************************
 *@file souce.c
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
 #include "souce.h"
 #include <string.h>
 
 void SOUNCE_Init()
 {
		UARTx_Init(SOUCE_PER);
 }
 
 void Sounce_Composition(const char *str,uint8_t cmd)
 {
	 uint16_t str_lenth=0;
	 struct SOUNCE_DEF sounce_def;
	 str_lenth=sizeof(str)+2;
	 sounce_def.head=0xFD;
	 sounce_def.lenth_h=((str_lenth>>8)&0x00ff);
	 sounce_def.lenth_l=str_lenth & 0x00ff;
	 sounce_def.comander=cmd;
	 sounce_def.style=STRING_STYLE;
	 memcpy(sounce_def.strinbuff,str,str_lenth-2);
	 USARTx_SendBytes(SOUCE_PER,&sounce_def.head,str_lenth+3);
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