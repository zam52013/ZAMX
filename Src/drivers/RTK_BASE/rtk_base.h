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
	 
	#define RTCM_BUFF_LEN 300
	 #define RTCM_MESG_CNT 7
	 #define NOWALT
	 
	 #ifdef NOWALT
	 /* MSM4*/
	 #define RKT_REF_DESCRIP "log com2 rtcm1006 ontime 10"
	 #define RKT_ANT_DESCRIP "log com2 rtcm1033 ontime 10"
	 #define RKT_CABLE_GPS "log com2 rtcm1074 ontime 1"
	 #define RKT_CABLE_GLONASS "log com2 rtcm1084 ontime 1"
	 #define RKT_CABLE_GALI "log com2 rtcm1094 ontime 1"
	 #define RKT_CABLE_QZSS "log com2 rtcm1114 ontime 1"
	 #define RKT_CABLE_BDS "log com2 rtcm1124 ontime 1"
	 #define RTK_SERIA_CONFIG "serialconfig com2 115200 n 8 1 n on"
	 #define RTK_RTCM_CONFIG_ON "interfacemode com2 none rtcmv3"
	 //#define RTK_POSITION_CONFIG "fix position 51.1136 -114.0435 1059.4"
	 #define RTK_ULOG	"unlogall"
	 
	 #else
	 
	 #define RKT_REF_DESCRIP "$PASHS,RT3,1006,B,10"
	 #define RKT_ANT_DESCRIP "$PASHS,RT3,1033,B,10"
	 #define RKT_CABLE_GPS "$PASHS,RT3,1074,B,1"
	 #define RKT_CABLE_GLONASS "$PASHS,RT3,1084,B,1"
	  #define RKT_CABLE_GALI "$PASHS,RT3,1094,B,1"
		#define RKT_CABLE_QZSS "$PASHS,RT3,1114,B,1"
	 #define RKT_CABLE_BDS "$PASHS,RT3,1124,B,1"
	 //#define RTK_POSITION_CONFIG "$PASHS,POS,5542.00,N,3737.54,E,205.15"
	 
	 
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
	double utc;				//UTC 时间
	double lat;				//纬度
	char   lat_dir;			//纬度方向
	double lon;				//经度
	char   lon_dir;			//经度方向
	char   quality;			//定位质量
	char   sats;			//解算卫星个数
	float  hdop;			//水平精度
	float  alt;				//天线高度
	char   a_units;			//单位
	float  undulation;		//大地水准面高程差
	char   u_units;			//单位
	char   age;				//差分标号
}gpgga;

typedef struct
{
	double latitude;				//纬度
	double longitude;				//经度
	double hgt;				//高程
}rtkposa;

	 extern RTCM_MESG rtcm_msg[RTCM_MESG_CNT];
	 void RTK_BASE_Init(void);
	 
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