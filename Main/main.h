/****************************************************************************
 *@file main.h
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

#ifndef __MAIN_H
#define __MAIN_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

 #if SYSTEM_SUPPORT_OS
#include "includes.h"					//支持OS时，使用	  
#endif

#ifdef USE_USB_OTG_FS
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#endif

/****************perph***************************/
#include "fm25v.h"
#include "key.h"
#include "mpu9250.h"
#include "souce.h"
#include "pow_ctr.h"
#include "led.h"
#include "rtk_base.h"
#include "esp32.h"
#include "radio.h"
/***********************************************/

/******************lib**************************/
#include "crc24.h"
/***********************************************/

/****************UCOSII TASK*************************/
//task init
#define START_TASK_PRIO                 25 //设置任务优先级
#define START_STK_SIZE                  64  //设置任务堆栈大小
void start_task(void *pdata);			//任务函数

//led task
#define LED_TASK_PRIO                 18 //设置任务优先级
#define LED_STK_SIZE                  64  //设置任务堆栈大小
void led_task(void *pdata);				//任务函数

//KEY task
#define KEY_TASK_PRIO                 5 //设置任务优先级
#define KEY_STK_SIZE                  64  //设置任务堆栈大小
void key_task(void *pdata);				//任务函数

//OEM get rtcm task
#define OEM_RTCM_TASK_PRIO                 8 //设置任务优先级
#define OEM_RTCM_STK_SIZE                  64  //设置任务堆栈大小
void oem_rtcm_task(void *pdata);				//任务函数

//OEM get date task
#define RTK_DATE_TASK_PRIO                 9 //设置任务优先级
#define RTK_DATE_STK_SIZE                  64  //设置任务堆栈大小
void rtk_date_task(void *pdata);				//任务函数

//ESP get date task
#define ESP_DATE_TASK_PRIO                 10 //设置任务优先级
#define ESP_DATE_STK_SIZE                  64  //设置任务堆栈大小
void esp_date_task(void *pdata);				//任务函数
/***************************************************/


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT FEIMA *****END OF FILE****/


