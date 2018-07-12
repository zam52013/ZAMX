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
 #include "esp32.h"
 
 	#ifdef SYSTEM_SUPPORT_OS
	 OS_EVENT *ESP_DATE_Semp;
	#endif
	
 ESP_MESG esp_mesg;
 
 void Wifi_Esp_Init(void)
 {
		UARTx_Init(ESP_PER);
		memset(&esp_mesg.esp__date_flag,0,sizeof(ESP_MESG));
 }
 
 void ESP_IRTHandler(void)
 {
	uint8_t res = 0;
	 
	#ifdef SYSTEM_SUPPORT_OS
	OSIntEnter();
	#endif
	 
	if(USART_GetITStatus(ESP_UART, USART_IT_RXNE) != RESET) 
	{
		res = USART_ReceiveData(ESP_UART);
		if(esp_mesg.esp__date_flag==0)
		{
			esp_mesg.esp_date_buff[esp_mesg.esp_date_lenth]=res;
			esp_mesg.esp_date_lenth++;
		}
	}
	
	else if(USART_GetITStatus(ESP_UART, USART_IT_IDLE) != RESET) 
	{
		esp_mesg.esp__date_flag=1;
		ESP_UART->SR;
    		ESP_UART->DR;
		#ifdef SYSTEM_SUPPORT_OS
		OSSemPost(ESP_DATE_Semp);
		#endif
	}
		 #ifdef SYSTEM_SUPPORT_OS
	OSIntExit();
	#endif 
 }
 
 void Clean_ESP_date(void)
 {
	memset(&esp_mesg.esp__date_flag,0,sizeof(ESP_MESG));
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
 