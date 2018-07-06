/****************************************************************************
 *@file times.c
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
 * @date 2018-07-03
 * 
 * 
 ****************************************************************************/
 
 /* Includes ------------------------------------------------------------------*/
 
 #include "times.h"	 
 #if SYSTEM_SUPPORT_OS
#include "includes.h"					//֧��OSʱ��ʹ��	  
#endif

#if SYSTEM_SUPPORT_OS

#ifdef 	OS_CRITICAL_METHOD						//OS_CRITICAL_METHOD������,˵��Ҫ֧��UCOSII				
#define delay_osrunning		OSRunning			//OS�Ƿ����б��,0,������;1,������
#define delay_ostickspersec	OS_TICKS_PER_SEC	//OSʱ�ӽ���,��ÿ����ȴ���
#define delay_osintnesting 	OSIntNesting		//�ж�Ƕ�׼���,���ж�Ƕ�״���
#endif

#endif
static u32 fac_ms=0;//ms 
 
unsigned char Time_statr_flag=0;	
unsigned char Time_out_flag=0;		
unsigned int Time_wait_cnt=0;		
uint16_t frameCounter = 0; 
time_flag TIME_FLAG; 


void delay_osschedlock(void)
{
#ifdef CPU_CFG_CRITICAL_METHOD   			//ʹ��UCOSIII
	OS_ERR err; 
	OSSchedLock(&err);						//UCOSIII�ķ�ʽ,��ֹ���ȣ���ֹ���us��ʱ
#else										//����UCOSII
	OSSchedLock();							//UCOSII�ķ�ʽ,��ֹ���ȣ���ֹ���us��ʱ
#endif
}

void delay_osschedunlock(void)
{	
#ifdef CPU_CFG_CRITICAL_METHOD   			//ʹ��UCOSIII
	OS_ERR err; 
	OSSchedUnlock(&err);					//UCOSIII�ķ�ʽ,�ָ�����
#else										//����UCOSII
	OSSchedUnlock();						//UCOSII�ķ�ʽ,�ָ�����
#endif
}

void delay_ostimedly(u32 ticks)
{
#ifdef CPU_CFG_CRITICAL_METHOD
	OS_ERR err; 
	OSTimeDly(ticks,OS_OPT_TIME_PERIODIC,&err);//UCOSIII��ʱ��������ģʽ
#else
	OSTimeDly(ticks);						//UCOSII��ʱ
#endif 
}
/*-------------------------------------- 

 ;-------------------------------------*/ 
 static void time_tick() 
 { 
 	static unsigned int Time_tic_cnt=0; 
 	if(Time_statr_flag) 
 	{ 
			if(Time_tic_cnt>=Time_wait_cnt) 
			{ 
 			Time_out_flag=1; 
		} 
 			Time_tic_cnt++; 
 	} 
 	else 
 	{ 
 		Time_tic_cnt=0; 
 	} 
 } 
/*-------------------------------------- 

 ;-------------------------------------*/ 
static void times_Flag() 
 { 
 	frameCounter++; 
 	TIME_FLAG.time_sub.flag_1Khz=TRUE; 
   if (frameCounter > FRAME_COUNT) 
   { 
     	frameCounter = 1; 
 	} 
 	if ((frameCounter % COUNT_500HZ) == 0) 
 	{ 
 		TIME_FLAG.time_sub.flag_500hz=TRUE; 
 	} 
 	if ((frameCounter % COUNT_100HZ) == 0) 
 	{ 
 		TIME_FLAG.time_sub.flag_100hz=TRUE; 
 	} 
 	if ((frameCounter % COUNT_50HZ) == 0) 
 	{ 
 		TIME_FLAG.time_sub.flag_50hz=TRUE; 
 	} 
 	if ((frameCounter % COUNT_10HZ) == 0) 
 	{ 
 		TIME_FLAG.time_sub.flag_10hz=TRUE; 
 	} 
 	if ((frameCounter % COUNT_5HZ) == 0) 
 	{ 
 		TIME_FLAG.time_sub.flag_5hz=TRUE; 
 	} 
 	if ((frameCounter % COUNT_2HZ) == 0) 
 	{ 
 		TIME_FLAG.time_sub.flag_2hz=TRUE; 
 	} 
 	if ((frameCounter % COUNT_1HZ) == 0) 
 	{ 
 		TIME_FLAG.time_sub.flag_1hz=TRUE; 
	} 
	if ((frameCounter % COUNT_0_5HZ) == 0) 
 	{ 
 		TIME_FLAG.time_sub.flag_0_5hz=TRUE; 
 	} 
 } 
void SysTick_Handler(void) 
 {				    
 	fac_ms--; 
 	time_tick(); 
 	times_Flag();
	if(delay_osrunning==1)					//OS��ʼ����,��ִ�������ĵ��ȴ���
	{
		OSIntEnter();						//�����ж�
		OSTimeTick();       				//����ucos��ʱ�ӷ������               
		OSIntExit();       	 				//���������л����ж�
	}	 
 } 
