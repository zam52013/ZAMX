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
	#ifdef SYSTEM_SUPPORT_OS
	#include "includes.h"
	#endif 

 gpgga GPGGA;
 rtkposa RTKPOSA;
 
 RTCM_MESG rtcm_msg[RTCM_MESG_CNT];
 static uint8_t RTCM_BUFF[5];
 void RTK_BASE_Init(void)
 {
		UARTx_Init(RTK_DAT);
		UARTx_Init(RTK_RTCM);
 }
 
 void RTK_RTCM_IRTHandler(void)
 {
		uint8_t res = 0;
	 static uint8_t statu=0;
	 static uint16_t Char_len=0;
	 static uint16_t LENTH_STR=0;
	 static uint16_t msg_lenth=0;
	 static uint8_t ID_MES=0;
	 unsigned int msg_id;
	 
		#ifdef SYSTEM_SUPPORT_OS
	OSIntEnter();
	#endif
	 if(USART_GetITStatus(RTK_RTCM_USART, USART_IT_RXNE) != RESET) 
	{
		res = USART_ReceiveData(RTK_RTCM_USART);
		switch(statu)
		{
			case 0:
				if(res==0xD3)
				{
					RTCM_BUFF[0]=0xD3;
					Char_len=0;
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
				if(LENTH_STR<(Char_len+3))
				{
					rtcm_msg[ID_MES].rtcm_buff[LENTH_STR+2]=res;
					LENTH_STR++;
				}
				else
				{
						rtcm_msg[ID_MES].rtcm_buff[LENTH_STR+2]=res;
						rtcm_msg[ID_MES].rtcm_flag=TRUE;
						statu=0;
				}
				break;
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