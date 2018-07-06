/****************************************************************************
 *@file souce.h
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
 
 #ifndef __SOUCE_H
 #define __SOUCE_H
 
   #ifdef __cplusplus
 extern "C" {
	#endif
	 
	 #include "board.h"
	 
	 #define STRING_STYLE 0x00			/***GB2312**/
//	 #define STRING_STYLE 0x01			/**GBK***/
//	 #define STRING_STYLE 0x02			/**BIG5***/
//	 #define STRING_STYLE 0x03			/**UNICODE***/
	 
	 struct SOUNCE_DEF
	 {
			uint8_t head;
			uint8_t lenth_h;
			uint8_t lenth_l;
			uint8_t comander;
			uint8_t style;
			uint8_t strinbuff[100];
	 };
	 
	 void SOUNCE_Init();
	 void Sounce_Composition(const char *str,uint8_t cmd);
	 
	 	 	#ifdef __cplusplus
	}
#endif
	
	
 #endif
 