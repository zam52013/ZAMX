/****************************************************************************
 *@file times.h
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
 #ifndef __TIMES_H
 #define __TIMES_H
 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx.h"

 extern unsigned char Time_statr_flag;	//if true star time
 extern unsigned char Time_out_flag;		//if true time out
 extern unsigned int Time_wait_cnt;		//time cnt
 
 #define FALSE 0 
 #define TRUE  1 
 
 #define FRAME_COUNT   2000 
 #define COUNT_500HZ   2         // Number of 1000 Hz frames for 500 Hz Loop 
 #define COUNT_100HZ   10        // Number of 1000 Hz frames for 100 Hz Loop 
 #define COUNT_50HZ    20        // Number of 1000 Hz frames for  50 Hz Loop 
 #define COUNT_10HZ    100       // Number of 1000 Hz frames for  10 Hz Loop 
 #define COUNT_5HZ     200       // Number of 1000 Hz frames for   5 Hz Loop 
 #define COUNT_2HZ     500       // Number of 1000 Hz frames for   2 Hz Loop 
 #define COUNT_1HZ     1000      // Number of 1000 Hz frames for   1 Hz Loop 
 #define COUNT_0_5HZ     2000      // Number of 1000 Hz frames for   1 Hz Loop 
 
 
 
 
 typedef union 
 {  
	unsigned int time_all; 
   struct 
 	{ 
 		unsigned flag_1Khz; 
 		unsigned flag_500hz; 
 		unsigned flag_100hz; 
 		unsigned flag_50hz; 
 		unsigned flag_10hz; 
 		unsigned flag_5hz; 
 		unsigned flag_2hz; 
 		unsigned flag_1hz; 
 		unsigned flag_0_5hz; 
 	}time_sub; 
 }time_flag; 
 
 
 extern time_flag TIME_FLAG; 
 
 
 void Delay_ms(u32 num); 

 #endif
 
 
 
 
 
 