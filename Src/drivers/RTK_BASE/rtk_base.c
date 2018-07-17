/****************************************************************************
 *@file rtkbase.c
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
 #include "rtk_base.h"
 #include <stdlib.h>
#include <string.h>
#include "mathlib.h"
#include <math.h>

#define RTK_CMD_CNT 100

 gpgga GPGGA;
 rtkposa RTKPOSA;
 
 RTCM_MESG rtcm_msg[RTCM_MESG_CNT];
 static uint8_t RTCM_BUFF[5];
 
 RTK_DATE_MESG rtk_date_mesg;
 
 #ifdef SYSTEM_SUPPORT_OS
 OS_EVENT *OEM_RTCM_Semp;
 OS_EVENT *RTK_DATE_Semp;
 #endif 

  static unsigned char RTK_TIME_START_FLAG=0;
 static unsigned char  RTK_TIME_OUT_FLAG=0;
static unsigned char rtk_timecnt=0;
static unsigned char rtk_wait_time=0;


 static char rtk_cmd_comander[RTK_CMD_CNT];
 static unsigned char RTK_CMD_FLAG=0;
  static unsigned char rtk_cmd_date_lenth=0;

 void RTK_BASE_Init(void)
 {
	unsigned char i=0;
	UARTx_Init(RTK_DAT);
	UARTx_Init(RTK_RTCM);
	for(i=0;i<RTCM_MESG_CNT;i++)
	 {
		memset(&rtcm_msg[i].rtcm_flag,0,sizeof(RTCM_MESG));
	 }
	memset(&rtk_date_mesg.rtk_date_flag,0,sizeof(RTK_DATE_MESG));
 }


void RTK_tick_time(void)
{
	if(RTK_TIME_START_FLAG)
	{
		if(rtk_timecnt>rtk_wait_time)
		{
			RTK_TIME_OUT_FLAG=1;
		}
		rtk_timecnt++;
	}
	else
	{
		rtk_timecnt=0;
	}
}
 static unsigned char RTK_find_string(char *str)
{
	if(NULL == str)
	{
		return 0;
	}
  	if(strstr(&rtk_cmd_comander[0],str) != NULL)
	  return 1;
	else
		return 0;
 }

 static unsigned char RTK_send_cmd(char *str,char *recive_str,unsigned char wait_time)
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
		RTK_TIME_START_FLAG=1;
		rtk_timecnt=0;
		rtk_wait_time=wait_time;
		RTK_TIME_OUT_FLAG=0;
		RTK_CMD_FLAG=1;
		rtk_cmd_date_lenth=0;
	}
	memset(&rtk_cmd_comander[0],0,RTK_CMD_CNT);
	UART_SendString(RTK_DAT,str);
	while(1)
	{
		if(RTK_find_string(recive_str))
		{
			ret=0;
			break;
		}
		if(RTK_TIME_OUT_FLAG)
		{
			ret=1;
			break;
		}
	}
	memset(&rtk_cmd_comander[0],0,RTK_CMD_CNT);
	RTK_TIME_START_FLAG=0;
	RTK_TIME_OUT_FLAG=0;
	RTK_CMD_FLAG=0;
	return ret;

 }
  unsigned char OEM_Init_output(void)
{
	OSTimeDlyHMSM(0,0,4,0);//wait 3 s
	#ifdef NOWALT
	
	#else
	if(RTK_send_cmd(RTK_SNS_CMD,"ACK",2)!=0)
	{
		#ifdef Debug
		DEBUG("RTK sns erro!\r\n");
		#endif
		return 1;
	}
	if(RTK_send_cmd(RTK_OUT_A_OFF,"ACK",2)!=0)
	{
		#ifdef Debug
		DEBUG("RTK A out erro!\r\n");
		#endif
		return 1;
	}
	if(RTK_send_cmd(RTK_OUT_B_OFF,"ACK",2)!=0)
	{
		#ifdef Debug
		DEBUG("RTK B out erro!\r\n");
		#endif
		return 1;
	}
	if(RTK_send_cmd(RTK_OUT_GGA,"ACK",2)!=0)
	{
		#ifdef Debug
		DEBUG("RTK gga out erro!\r\n");
		#endif
		return 1;
	}
	#endif
	return 0;
	
 }
unsigned char OEM_RTK_BASE(void)
{
	#ifdef NOWALT
	
	#else
	if(RTK_send_cmd(RTK_REF_DESCRIP,"ACK",2)!=0)
	{
		#ifdef Debug
		DEBUG("RTK 1006 erro!\r\n");
		#endif
		return 1;
	}
	if(RTK_send_cmd(RTK_ANT_DESCRIP,"ACK",2)!=0)
	{
		#ifdef Debug
		DEBUG("RTK 1033 erro!\r\n");
		#endif
		return 1;
	}
	if(RTK_send_cmd(RTK_CABLE_GPS,"ACK",2)!=0)
	{
		#ifdef Debug
		DEBUG("RTK 1074 erro!\r\n");
		#endif
		return 1;
	}
	if(RTK_send_cmd(RTK_CABLE_GLONASS,"ACK",2)!=0)
	{
		#ifdef Debug
		DEBUG("RTK 1084 erro!\r\n");
		#endif
		return 1;
	}
		if(RTK_send_cmd(RTK_CABLE_BDS,"ACK",2)!=0)
	{
		#ifdef Debug
		DEBUG("RTK 1124 erro!\r\n");
		#endif
		return 1;
	}
	#endif
	return 0;
}
 void RTK_RTCM_IRTHandler(void)
 {
		uint8_t res = 0;
	 static uint8_t statu=0;
	 static uint16_t LENTH_STR=0;
	 static uint16_t msg_lenth=0;
	 static uint8_t ID_MES=0;
	 unsigned int msg_id;
	 
	#ifdef SYSTEM_SUPPORT_OS
	OSIntEnter();
	#endif
	 if(USART_GetITStatus(RTK_RTCM_UART, USART_IT_RXNE) != RESET) 
	{
		res = USART_ReceiveData(RTK_RTCM_UART);
		switch(statu)
		{
			case 0:
				if(res==0xD3)
				{
					RTCM_BUFF[0]=0xD3;
					LENTH_STR=0;
					statu=1;	
				}
				break;
			case 1:
				RTCM_BUFF[1]=res;
				statu=2;
				break;
			case 2:
				RTCM_BUFF[2]=res;
				msg_lenth=(RTCM_BUFF[1]<<8) + RTCM_BUFF[2];
				if(msg_lenth>=RTCM_BUFF_LEN)
				{
					statu=1;
				}
				else
				{	
					statu=3;
				}
				break;
			case 3:
				RTCM_BUFF[3]=res;
				statu=4;
				break;
			case 4:
				RTCM_BUFF[4]=res;
				msg_id=RTCM_BUFF[3]*16+RTCM_BUFF[4]/16;
				switch(msg_id)
				{
					case 1006:
						ID_MES=MSG_ID1006;
						rtcm_msg[ID_MES].head=RTCM_BUFF[0];
						rtcm_msg[ID_MES].lenth_h=RTCM_BUFF[1];
						rtcm_msg[ID_MES].lenth_l=RTCM_BUFF[2];
						rtcm_msg[ID_MES].rtcm_buff[0]=RTCM_BUFF[3];
						rtcm_msg[ID_MES].rtcm_buff[1]=RTCM_BUFF[4];
						statu=5;
						break;
					case 1033:
						ID_MES=MSG_ID1033;
						rtcm_msg[ID_MES].head=RTCM_BUFF[0];
						rtcm_msg[ID_MES].lenth_h=RTCM_BUFF[1];
						rtcm_msg[ID_MES].lenth_l=RTCM_BUFF[2];
						rtcm_msg[ID_MES].rtcm_buff[0]=RTCM_BUFF[3];
						rtcm_msg[ID_MES].rtcm_buff[1]=RTCM_BUFF[4];
						statu=5;
						break;
					case 1074:
						ID_MES=MSG_ID1074;
						rtcm_msg[ID_MES].head=RTCM_BUFF[0];
						rtcm_msg[ID_MES].lenth_h=RTCM_BUFF[1];
						rtcm_msg[ID_MES].lenth_l=RTCM_BUFF[2];
						rtcm_msg[ID_MES].rtcm_buff[0]=RTCM_BUFF[3];
						rtcm_msg[ID_MES].rtcm_buff[1]=RTCM_BUFF[4];
						statu=5;
						break;
					case 1084:
						ID_MES=MSG_ID1084;
						rtcm_msg[ID_MES].head=RTCM_BUFF[0];
						rtcm_msg[ID_MES].lenth_h=RTCM_BUFF[1];
						rtcm_msg[ID_MES].lenth_l=RTCM_BUFF[2];
						rtcm_msg[ID_MES].rtcm_buff[0]=RTCM_BUFF[3];
						rtcm_msg[ID_MES].rtcm_buff[1]=RTCM_BUFF[4];
						statu=5;
						break;
					case 1094:
						ID_MES=MSG_ID1094;
						rtcm_msg[ID_MES].head=RTCM_BUFF[0];
						rtcm_msg[ID_MES].lenth_h=RTCM_BUFF[1];
						rtcm_msg[ID_MES].lenth_l=RTCM_BUFF[2];
						rtcm_msg[ID_MES].rtcm_buff[0]=RTCM_BUFF[3];
						rtcm_msg[ID_MES].rtcm_buff[1]=RTCM_BUFF[4];
						statu=5;
						break;
					case 1114:
						ID_MES=MSG_ID1114;
						rtcm_msg[ID_MES].head=RTCM_BUFF[0];
						rtcm_msg[ID_MES].lenth_h=RTCM_BUFF[1];
						rtcm_msg[ID_MES].lenth_l=RTCM_BUFF[2];
						rtcm_msg[ID_MES].rtcm_buff[0]=RTCM_BUFF[3];
						rtcm_msg[ID_MES].rtcm_buff[1]=RTCM_BUFF[4];
						statu=5;
						break;
					case 1124:
						ID_MES=MSG_ID1124;
						rtcm_msg[ID_MES].head=RTCM_BUFF[0];
						rtcm_msg[ID_MES].lenth_h=RTCM_BUFF[1];
						rtcm_msg[ID_MES].lenth_l=RTCM_BUFF[2];
						rtcm_msg[ID_MES].rtcm_buff[0]=RTCM_BUFF[3];
						rtcm_msg[ID_MES].rtcm_buff[1]=RTCM_BUFF[4];
						statu=5;
						break;
					default:
						statu=0;
						break;
				}
				break;
			case 5:
				if(LENTH_STR<(msg_lenth))
				{
					rtcm_msg[ID_MES].rtcm_buff[LENTH_STR+2]=res;
					LENTH_STR++;
				}
				else
				{
						rtcm_msg[ID_MES].rtcm_buff[LENTH_STR+2]=res;
						rtcm_msg[ID_MES].rtcm_flag=TRUE;
						statu=0;
						OSSemPost(OEM_RTCM_Semp);
				}
				break;
		}
	}
	 #ifdef SYSTEM_SUPPORT_OS
	OSIntExit();
	#endif 
 }

 void RTK_DAT_IRTHandler(void)
 {
	 uint8_t res = 0;
	 static uint8_t statu=0;
	#ifdef SYSTEM_SUPPORT_OS
	OSIntEnter();
	#endif
	 
	 if(USART_GetITStatus(RTK_DAT_UART, USART_IT_RXNE)!=RESET)
	 {
			res =USART_ReceiveData(RTK_DAT_UART);
			if(RTK_CMD_FLAG)
			{
				rtk_cmd_comander[rtk_cmd_date_lenth]=res;
				rtk_cmd_date_lenth++;
				if(rtk_cmd_date_lenth>=RTK_CMD_CNT)
				{
					rtk_cmd_date_lenth=0;
				}
				
			}
			else
			{
				if(rtk_date_mesg.rtk_date_flag==0)
				{
					rtk_date_mesg.rtcm_buff[rtk_date_mesg.rtk_date_lenth]=res;
					rtk_date_mesg.rtk_date_lenth++;
					if(rtk_date_mesg.rtk_date_lenth>(RTK_BUFF_LEN-1))
					{
						rtk_date_mesg.rtk_date_lenth=RTK_BUFF_LEN-1;//预留一个位防止越界
					}
					/*switch(statu)
					{
						case 0:
							memset(rtk_date_mesg.rtcm_buff,0,RTK_BUFF_LEN);
							rtk_date_mesg.rtk_date_lenth=0;
							statu=1;
							rtk_date_mesg.rtcm_buff[rtk_date_mesg.rtk_date_lenth]=res;
							rtk_date_mesg.rtk_date_lenth++;
							break;
						case 1:
							if(res==0x0d)
							{
								statu=2;
								rtk_date_mesg.rtcm_buff[rtk_date_mesg.rtk_date_lenth]=res;
								rtk_date_mesg.rtk_date_lenth++;
							}
							else
							{
								rtk_date_mesg.rtcm_buff[rtk_date_mesg.rtk_date_lenth]=res;
								rtk_date_mesg.rtk_date_lenth++;
							}
							if(rtk_date_mesg.rtk_date_lenth>RTK_BUFF_LEN)
							{
								statu=0;
							}
							break;
						case 2:
							if(res==0x0a)
							{
								rtk_date_mesg.rtcm_buff[rtk_date_mesg.rtk_date_lenth]=res;
								rtk_date_mesg.rtk_date_lenth++;
								statu=0;
								rtk_date_mesg.rtk_date_flag=1;
								OSSemPost(RTK_DATE_Semp);
							}
							else
							{
								statu=0;
							}
							break;
							default:
								break;
					}*/
			}
				//else if(USART_GetITStatus(RTK_DAT_UART, USART_IT_IDLE)!=RESET)
		}
	 }
	 else if(USART_GetFlagStatus(RTK_DAT_UART, USART_FLAG_IDLE)!=RESET)
	{
			RTK_DAT_UART->SR;
			RTK_DAT_UART->DR;
			if(!RTK_CMD_FLAG)
			{
				rtk_date_mesg.rtk_date_flag=1;
				OSSemPost(RTK_DATE_Semp);
			}
	}
	#ifdef SYSTEM_SUPPORT_OS
	OSIntExit();
	#endif 
 }
 /********************************************************************

********************************************************************/
unsigned Nmea_parse_GGA(char* buffe,int buff_len)
{
	int i=6;
	if(buffe[i]==','&&buffe[++i]==',')
	{
		return 0;
	}
	GPGGA.utc=strtod(&buffe[i],NULL);
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.lat=strtod(&buffe[i],NULL);
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.lat_dir=buffe[i];
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.lon=strtod(&buffe[i],NULL);
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.lon_dir=buffe[i];
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.quality=atoi(&buffe[i]);
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.sats=atoi(&buffe[i]);
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.hdop=strtof(&buffe[i],NULL);
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.alt=strtof(&buffe[i],NULL);
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.a_units=buffe[i];
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.undulation=strtof(&buffe[i],NULL);
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.u_units=buffe[i];
	while(buffe[i++]!=',')
	{
		if(i>buff_len)
		{
			return 0;
		}
	}
	GPGGA.age=atoi(&buffe[i]);
	return 1;
}
/********************************************************************

********************************************************************/
unsigned Nmea_parse_PSRPOSA(char* buffe,int buff_len)
{
	char *str;
	int i=68;
	str=strchr(buffe,';');
	if(str)
	{
		while(*(str++)!=',')
		{
			i++;
			if(i>buff_len)
			{
				return 0;
			}
		}
		while(*(str++)!=',')
		{
			i++;
			if(i>buff_len)
			{
				return 0;
			}
		}
		RTKPOSA.latitude=strtod(str,NULL);
		while(*(str++)!=',')
		{
			i++;
			if(i>buff_len)
			{
				return 0;
			}
		}
		RTKPOSA.longitude=strtod(str,NULL);
		while(*(str++)!=',')
		{
			i++;
			if(i>buff_len)
			{
				return 0;
			}
		}
		RTKPOSA.hgt=strtod(str,NULL);
		return 1;
	}
	else
	{
		return 0;
	}
}
 double Get_Dis_m(double lat_tar,double lon_tar,double lat_get,double lon_get)
{
	return 6371004*acos(1-(pow((sin((90-lat_tar)*M_PI_F/180)*cos(lon_tar*M_PI_F/180)\
			-sin((90-lat_get)*M_PI_F/180)*cos(lon_get*M_PI_F/180)),2)+pow((sin((90-lat_tar)*M_PI_F/180)\
			*sin(lon_tar*M_PI_F/180)-sin((90-lat_get)*M_PI_F/180)*sin(lon_get*M_PI_F/180)),2)+pow((cos((90-\
			lat_tar)*M_PI_F/180)-cos((90-lat_get)*M_PI_F/180)),2))/2);
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