/****************************************************************************
 *@file rtkbase.h
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
 #ifndef __RTK_BASE_H
 #define __RTK_BASE_H
 
 
   #ifdef __cplusplus
 extern "C" {
	#endif
	 
	 #include "board.h"
	 #ifdef SYSTEM_SUPPORT_OS
#include "includes.h"
#endif 
	 
	 
	#define RTCM_BUFF_LEN 400
	 #define RTCM_MESG_CNT 7
	 #define RTK_BUFF_LEN 2048
	 
	 //#define NOWALT
	 
	 #ifdef NOWALT
	 /* MSM4*/
	 #define RTK_REF_DESCRIP "log com2 rtcm1006 ontime 10\r\n"
	 #define RTK_ANT_DESCRIP "log com2 rtcm1033 ontime 10\r\n"
	 #define RTK_CABLE_GPS "log com2 rtcm1074 ontime 1\r\n"
	 #define RTK_CABLE_GLONASS "log com2 rtcm1084 ontime 1\r\n"
	 #define RTK_CABLE_GALI "log com2 rtcm1094 ontime 1\r\n"
	 #define RTK_CABLE_QZSS "log com2 rtcm1114 ontime 1\r\n"
	 #define RTK_CABLE_BDS "log com2 rtcm1124 ontime 1\r\n"
	 #define RTK_SERIA_CONFIG "serialconfig com2 115200 n 8 1 n on\r\n"
	 #define RTK_RTCM_CONFIG_ON "interfacemode com2 none rtcmv3\r\n"
	 //#define RTK_POSITION_CONFIG "fix position 51.1136 -114.0435 1059.4\r\n"
	 #define RTK_ULOG	"unlogall\r\n"
	 
	 #else
	 #define RTK_SNS_CMD "$PASHS,SNS,SOL\r\n"
	 #define RTK_REF_DESCRIP "$PASHS,RT3,1006,B,ON,10\r\n"
	 #define RTK_ANT_DESCRIP "$PASHS,RT3,1033,B,ON,10\r\n"
	 #define RTK_CABLE_GPS "$PASHS,RT3,1074,B,ON,1\r\n"
	 #define RTK_CABLE_GLONASS "$PASHS,RT3,1084,B,ON,1\r\n"
	  #define RTK_CABLE_GALI "$PASHS,RT3,1094,B,ON,1\r\n"
	#define RTK_CABLE_QZSS "$PASHS,RT3,1114,B,ON,1\r\n"
	 #define RTK_CABLE_BDS "$PASHS,RT3,1124,B,ON,1\r\n"
	 #define RTK_OUT_GGA "$PASHS,NME,GGA,A,ON,1\r\n"
	 #define RTK_OUT_A_OFF "$PASHS,NME,ALL,A,OFF\r\n"
	 #define RTK_OUT_B_OFF "$PASHS,RT3,ALL,B,OFF\r\n"
	 //#define RTK_POSITION_CONFIG "$PASHS,POS,5542.00,N,3737.54,E,205.15\r\n"
	 
	 
	 #endif


typedef enum
{
	MSG_ID1006=0,
	MSG_ID1033,
	MSG_ID1074,
	MSG_ID1084,
	MSG_ID1094,
	MSG_ID1114,
	MSG_ID1124
}RTCM_MSG_ID;

typedef struct
{
	uint8_t rtcm_flag;
	uint8_t head;
	uint8_t lenth_h;
	uint8_t lenth_l;
	uint8_t rtcm_buff[RTCM_BUFF_LEN];
}RTCM_MESG;

typedef struct
{
	uint8_t rtk_date_flag;
	uint16_t rtk_date_lenth;
	uint8_t rtcm_buff[RTK_BUFF_LEN];
}RTK_DATE_MESG;


typedef struct
{
	double utc;				//UTC ʱ��
	double lat;				//γ��
	char   lat_dir;			//γ�ȷ���
	double lon;				//����
	char   lon_dir;			//���ȷ���
	char   quality;			//��λ����
	char   sats;			//�������Ǹ���
	float  hdop;			//ˮƽ����
	float  alt;				//���߸߶�
	char   a_units;			//��λ
	float  undulation;		//���ˮ׼��̲߳�
	char   u_units;			//��λ
	char   age;				//��ֱ��
}gpgga;

typedef struct
{
	double latitude;				//γ��
	double longitude;				//����
	double hgt;				//�߳�
}rtkposa;

	 #ifdef SYSTEM_SUPPORT_OS
	extern OS_EVENT *OEM_RTCM_Semp;
	extern OS_EVENT *RTK_DATE_Semp;	
	#endif

	 extern RTCM_MESG rtcm_msg[RTCM_MESG_CNT];
	extern RTK_DATE_MESG rtk_date_mesg;
	
	void	RTK_tick_time(void);
	 void RTK_BASE_Init(void);
	 unsigned char OEM_Init_output(void);
	 unsigned char OEM_RTK_BASE(void);
	 void rtcm_time_cnt(void);

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