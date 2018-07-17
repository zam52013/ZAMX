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

 #define CMD_CNT 100
 
	static unsigned char WIFI_DATE_FLAG=0;
	static unsigned int wifi_need_date_lenth=0;
	static unsigned int wifi_get_date_lenth=0;
	static  char cmd_comander[CMD_CNT];
	static unsigned char wifi_cmd_date_lenth=0;
	unsigned char WIFI_CHAN_ON_FLAG[WIF_CHAN_CNT];

	static unsigned char wifi_timer_start=0;
	static unsigned char wifi_timer_cnt=0;
	static unsigned char wifi_wait_time=0;
	static unsigned char TIME_OUT_FLAG=0;
 
 void Wifi_Esp_Init(void)
 {
		UARTx_Init(ESP_PER);
		memset(&esp_mesg.esp__date_flag,0,sizeof(ESP_MESG));
		memset(&WIFI_CHAN_ON_FLAG,0,WIF_CHAN_CNT);
 }

 static  void Clear_cmd_buff(void)
  {
	memset(cmd_comander, 0, CMD_CNT);
	wifi_cmd_date_lenth=0;
   }
 static  void ESP_uart_buff(uint8_t res)
   {
	char *ESP_CNT;
	if((res=='\n')||(res==':'))
	{
		if((ESP_CNT=strstr(cmd_comander,"+IPD"))!=NULL)
		{
			WIFI_DATE_FLAG=1;
			ESP_CNT+=4;
			esp_mesg.esp__chan=atoi(ESP_CNT);
			ESP_CNT+=3;
			wifi_need_date_lenth=atoi(ESP_CNT);
			if(wifi_need_date_lenth>ESP_BUFF_LEN)
			{
				wifi_need_date_lenth=ESP_BUFF_LEN-1;
			}
			wifi_get_date_lenth=0;
			
			Clear_cmd_buff();
		}
		else if((ESP_CNT=strstr(cmd_comander,"CONNECT"))!=NULL)
		{
			if(strstr(cmd_comander,"0")!=NULL)
			{
				WIFI_CHAN_ON_FLAG[0]=1;
				#ifdef Debug
				DEBUG("CONNECT 0!\r\n");
				#endif
			}
			else if(strstr(cmd_comander,"1")!=NULL)
			{
				WIFI_CHAN_ON_FLAG[1]=1;
					#ifdef Debug
				DEBUG("CONNECT 1!\r\n");
				#endif
			}
			else if(strstr(cmd_comander,"2")!=NULL)
			{
				WIFI_CHAN_ON_FLAG[2]=1;
					#ifdef Debug
				DEBUG("CONNECT 2!\r\n");
				#endif
			}
			else if(strstr(cmd_comander,"3")!=NULL)
			{
				WIFI_CHAN_ON_FLAG[3]=1;
					#ifdef Debug
				DEBUG("CONNECT 3!\r\n");
				#endif
			}
			Clear_cmd_buff();
		}
		else if((ESP_CNT=strstr(cmd_comander,"CLOSED"))!=NULL)
		{
			if(strstr(cmd_comander,"0")!=NULL)
			{
				WIFI_CHAN_ON_FLAG[0]=0;
					#ifdef Debug
				DEBUG("CLOSED 0!\r\n");
				#endif
			}
			else if(strstr(cmd_comander,"1")!=NULL)
			{
				WIFI_CHAN_ON_FLAG[1]=0;
					#ifdef Debug
				DEBUG("CLOSED 1!\r\n");
				#endif
			}
			else if(strstr(cmd_comander,"2")!=NULL)
			{
				WIFI_CHAN_ON_FLAG[2]=0;
					#ifdef Debug
				DEBUG("CLOSED 2!\r\n");
				#endif
			}
			else if(strstr(cmd_comander,"3")!=NULL)
			{
				WIFI_CHAN_ON_FLAG[3]=0;
					#ifdef Debug
				DEBUG("CLOSED 3!\r\n");
				#endif
			}
			Clear_cmd_buff();
		}
		else 
		{
			//Clear_cmd_buff();
		}
	}
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
		if(WIFI_DATE_FLAG==1)
		{
			esp_mesg.esp_date_buff[wifi_get_date_lenth]=res;
			wifi_get_date_lenth++;
			if(wifi_get_date_lenth>=wifi_need_date_lenth)
			{
				WIFI_DATE_FLAG=0;
				esp_mesg.esp_date_lenth=wifi_get_date_lenth;
				esp_mesg.esp_date_buff[wifi_get_date_lenth]='\0';
				esp_mesg.esp__date_flag=1;
				wifi_cmd_date_lenth=0;
				#ifdef SYSTEM_SUPPORT_OS
				OSSemPost(ESP_DATE_Semp);
				#endif
			}
		}
		else
		{
			cmd_comander[wifi_cmd_date_lenth]=res;
			wifi_cmd_date_lenth++;
			//USART_SendData(USART1,res);
			if(wifi_cmd_date_lenth>=(CMD_CNT-1))
			{
				wifi_cmd_date_lenth=0;
			}
			ESP_uart_buff(res);
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
 static unsigned char esp_find_string(char *str)
{
	if(NULL == str)
	{
		return 0;
	}
  	if(strstr(cmd_comander,str) != NULL)
	  return 1;
	else
		return 0;
 }
 char wifi_send_comd( char *str,char *recive_str,unsigned char wait_time)
{
	unsigned char ret=0;
	if(str==NULL)
	{
		return 0;
	}
	if((recive_str==NULL )||(wait_time<=0))
	{
		return 0;
	}
	else
	{
		wifi_timer_start=1;
		wifi_timer_cnt=0;
		wifi_wait_time=wait_time;
		TIME_OUT_FLAG=0;
	}
	Clear_cmd_buff();
	UART_SendString(ESP_PER,str);
	while(1)
	{
		if(!esp_find_string(recive_str))
		{
			
			if(strstr(cmd_comander,"ERROR") != NULL)
			{
				ret=1;
				break;
			}
		}
		else
		{
			ret=0;//have str
			break;
		}
		if(TIME_OUT_FLAG==1)
		{
			ret=2;
			break;
		}
	}
	Clear_cmd_buff();
	wifi_timer_start=0;
	TIME_OUT_FLAG=0;
	return ret;
}
 
unsigned char wifi_reg(void)
{
	unsigned char wifi_reg=1;
	OSTimeDlyHMSM(0,0,3,0);//wait 3 s
	while(wifi_reg)
	{
		if(wifi_send_comd(CMD_TEST,"OK",2)==0)
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
	if(wifi_send_comd(CMD_CLOSE_SR,"OK",2)!=0)
	{
		return 1;
	}
	if(wifi_send_comd(CMD_MODE,"OK",2)!=0)
	{
		#ifdef Debug
		DEBUG("mode set erro!\r\n");
		#endif
		return 1;
	}
	if(wifi_send_comd(CMD_MUX,"OK",2)!=0)
	{
		#ifdef Debug
		DEBUG("mux set erro!\r\n");
		#endif
		return 1;
	}
	if(wifi_send_comd(CMD_SEVER,"OK",2)!=0)
	{
		#ifdef Debug
		DEBUG("sever set erro!\r\n");
		#endif
		return 1;
	}
	if(wifi_send_comd(CMD_IP,"OK",2)!=0)
	{
		#ifdef Debug
		DEBUG("IP set erro!\r\n");
		#endif
		return 1;
	}
	if(wifi_send_comd("AT+CWSAP?\r\n","FM-BASE",3)!=0)
	{
		if(wifi_send_comd(CMD_SAP,"OK",3)!=0)
		{
			return 1;
		}
	}
	#ifdef Debug
	DEBUG("wifi init ok!\r\n");
	#endif
	return 0;
}

unsigned int wifi_soc_send(unsigned char soc,uint8_t *send_buffer,unsigned int length)
{
	char send_buff[2048];
	char connet_buff[30];
	memset(send_buff,0,sizeof(send_buff));
	if(length>=2048)
	{
		#ifdef Debug
		DEBUG("over send lenth!\r\n");
		#endif
		return 1;
	}
	sprintf(connet_buff,"AT+CIPSEND=%d,%d\r\n",soc,length);
	if(wifi_send_comd(connet_buff,">",2)!=0)
	{
		#ifdef Debug
		DEBUG("send cmd erro!\r\n");
		#endif
		//USARTx_SendBytes(ESP_PER,send_buffer,length);
		return 1;
	}
	sprintf(send_buff,"%s\32\0",send_buffer);
	if(wifi_send_comd(&send_buff[0],"OK",2)!=0)
	{
		#ifdef Debug
		DEBUG("send erro!\r\n");
		#endif
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
 
