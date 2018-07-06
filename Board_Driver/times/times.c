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
#include "includes.h"					//支持OS时，使用	  
#endif

#if SYSTEM_SUPPORT_OS

#ifdef 	OS_CRITICAL_METHOD						//OS_CRITICAL_METHOD定义了,说明要支持UCOSII				
#define delay_osrunning		OSRunning			//OS是否运行标记,0,不运行;1,在运行
#define delay_ostickspersec	OS_TICKS_PER_SEC	//OS时钟节拍,即每秒调度次数
#define delay_osintnesting 	OSIntNesting		//中断嵌套级别,即中断嵌套次数
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
#ifdef CPU_CFG_CRITICAL_METHOD   			//使用UCOSIII
	OS_ERR err; 
	OSSchedLock(&err);						//UCOSIII的方式,禁止调度，防止打断us延时
#else										//否则UCOSII
	OSSchedLock();							//UCOSII的方式,禁止调度，防止打断us延时
#endif
}

void delay_osschedunlock(void)
{	
#ifdef CPU_CFG_CRITICAL_METHOD   			//使用UCOSIII
	OS_ERR err; 
	OSSchedUnlock(&err);					//UCOSIII的方式,恢复调度
#else										//否则UCOSII
	OSSchedUnlock();						//UCOSII的方式,恢复调度
#endif
}

void delay_ostimedly(u32 ticks)
{
#ifdef CPU_CFG_CRITICAL_METHOD
	OS_ERR err; 
	OSTimeDly(ticks,OS_OPT_TIME_PERIODIC,&err);//UCOSIII延时采用周期模式
#else
	OSTimeDly(ticks);						//UCOSII延时
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
	if(delay_osrunning==1)					//OS开始跑了,才执行正常的调度处理
	{
		OSIntEnter();						//进入中断
		OSTimeTick();       				//调用ucos的时钟服务程序               
		OSIntExit();       	 				//触发任务切换软中断
	}	 
 } 
