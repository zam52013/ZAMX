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
 #include <string.h>
 
 	#ifdef SYSTEM_SUPPORT_OS
	 OS_EVENT *ESP_DATE_Semp;
	#endif
	
 ESP_MESG esp_mesg;

 static unsigned char wifi_timer_start=0;
 static unsigned char wifi_timer_cnt=0;
 static unsigned char wifi_wait_time=0;
  static unsigned char TIME_OUT_FLAG=0;
  static unsigned char POST_FLAG=0;
 
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
		USART_SendData(USART1,res);
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
		if(POST_FLAG)
		{
			#ifdef SYSTEM_SUPPORT_OS
			OSSemPost(ESP_DATE_Semp);
			#endif
		}
	}
	#ifdef SYSTEM_SUPPORT_OS
	OSIntExit();
	#endif 
 }

 void wifi_tick_time()
 {
	if(wifi_timer_start)
	{
		if(wifi_timer_cnt>wifi_wait_time)
		{
			TIME_OUT_FLAG=1;
		}
		wifi_timer_cnt++;
	}
	else
	{
		wifi_timer_cnt=0;
	}
 }
 void Clean_ESP_date(void)
 {
	memset(&esp_mesg.esp__date_flag,0,sizeof(ESP_MESG));
 }
 char wifi_send_comd( char *str,char *recive_str,unsigned char wait_time)
{
	unsigned char ret=0;
	if(str==NULL)
	{
		return -1;
	}
	Clean_ESP_date();
	POST_FLAG=0;
	if((recive_str==NULL )||(wait_time<=0))
	{
		return 0;
	}
	else
	{
		wifi_timer_start=1;
		wifi_wait_time=wait_time;
		TIME_OUT_FLAG=0;
	}
	UART_SendString(ESP_PER,str);
	while(1)
	{
		if(esp_mesg.esp__date_flag)
		{
			if(strstr(esp_mesg.esp_date_buff,recive_str) != NULL)
			{
				ret=0;
				break;//have str
			}
			else
			{
				ret=1;
				break;
			}
			Clean_ESP_date();
		}
		if(TIME_OUT_FLAG==1)
		{
			ret=2;
			break;
		}
	}
	Clean_ESP_date();
	POST_FLAG=1;
	wifi_timer_start=0;
	TIME_OUT_FLAG=0;
	return ret;
}
 
unsigned char wifi_reg(void)
{
	unsigned char wifi_reg=1;
	OSTimeDlyHMSM(0,0,3,0);//wait 10 s
	while(wifi_reg)
	{
		if(wifi_send_comd(CMD_TEST,"OK",1)==0)
		{
			wifi_reg=0;
			break;
		}
		else
		{
			wifi_send_comd("AT+RST\r\n","ready",5);
			#ifdef Debug
			DEBUG("wifi erro!\r\n");
			#endif
		}
	}
	if(wifi_send_comd(CMD_CLOSE_SR,"OK",1)!=0)
	{
		return 1;
	}
	if(wifi_send_comd(CMD_MODE,"OK",1)!=0)
	{
		#ifdef Debug
		DEBUG("mode set ok!\r\n");
		#endif
		return 1;
	}
	if(wifi_send_comd(CMD_MUX,"OK",1)!=0)
	{
		#ifdef Debug
		DEBUG("mux set ok!\r\n");
		#endif
		return 1;
	}
	if(wifi_send_comd(CMD_SEVER,"OK",2)!=0)
	{
		return 1;
	}
	if(wifi_send_comd(CMD_IP,"OK",2)!=0)
	{
		return 1;
	}
	if(wifi_send_comd("AT+CWSAP?\r\n","FM-BASE",3)!=0)
	{
		if(wifi_send_comd(CMD_SAP,"OK",3)!=0)
		{
			return 1;
		}
	}
	return 0;
}

unsigned int wifi_soc_send(unsigned char soc,uint8_t *send_buffer,unsigned int length)
{
	char send_buff[1024];
	char connet_buff[100];
	memset(send_buff,0,sizeof(send_buff));
	sprintf(connet_buff,"AT+CIPSEND=%d,%d",soc,length);
	if(wifi_send_comd(connet_buff,">",1)!=0)
	{
		#ifdef Debug
		DEBUG("send erro!\r\n");
		#endif
		return 1;
	}
	sprintf(send_buff,"%s\32\0",send_buffer);
	if(wifi_send_comd(send_buff,"SEND OK",2)!=0)
	{
		return 1;
	}
	return 0;
	
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
 
