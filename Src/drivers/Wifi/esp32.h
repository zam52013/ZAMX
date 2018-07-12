/****************************************************************************
 *@file esp32.h
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
 * @date 2018-07-09
 * 
 * 
 ****************************************************************************/
 #ifndef __ESP32_H
 #define __ESP32_H
 
   #ifdef __cplusplus
 extern "C" {
	#endif
	 
	 #include "board.h" 
	 	 #ifdef SYSTEM_SUPPORT_OS
#include "includes.h"
#endif
	 
	 #define ESP_BUFF_LEN 1024
	 
	 #define CMD_TEST "AT\r\n"
	 #define CMD_CLOSE_SR "ATE0\r\n"
	 #define CMD_MODE "AT+CWMODE=2\r\n"
	 #define CMD_MUX "AT+CIPMUX=1\r\n"
	 #define CMD_SEVER "AT+CIPSERVER=1,8080\r\n"
	 #define CMD_SAP "AT+CWSAP=\"FM-BASE\",\"12345678\",1,3\r\n"
	 #define CMD_SEND "AT+CIPSEND=0,6\r\n" 
	 #define CMD_IP "AT+CIPAP=\"135.154.2.1\",\"135.154.2.1\",\"255.255.255.0\""
	 
	 typedef struct
	 {
			uint8_t esp__date_flag;
			uint16_t esp_date_lenth;
			uint8_t esp_date_buff[];
	 }ESP_MESG;
	 
	 #ifdef SYSTEM_SUPPORT_OS
	extern OS_EVENT *ESP_DATE_Semp;
	#endif
	 extern ESP_MESG esp_mesg;
	 
	 void Wifi_Esp_Init(void);
	 void Clean_ESP_date(void);
	 
	 	#ifdef __cplusplus
	}
#endif
 
 #endif
 
 
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