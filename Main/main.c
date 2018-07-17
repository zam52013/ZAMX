/****************************************************************************
 *@file main.c
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
#include "main.h"
#include <string.h>


/** @defgroup APP_VCP_Private_Variables
  * @{
  */ 

#ifdef USE_USB_OTG_FS

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
   
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

#endif 
/**
  * @}
  */ 

/*******************task stk set*************************/
OS_STK START_TASK_STK[START_STK_SIZE];			/*create task*/
OS_STK TICKIT_TASK_STK[TICKIT_STK_SIZE];			/*tickit task*/
OS_STK LED_TASK_STK[LED_STK_SIZE];				/*LED task*/
OS_STK KEY_TASK_STK[KEY_STK_SIZE];				/*KEY task*/
OS_STK OEM_RTCM_TASK_STK[OEM_RTCM_STK_SIZE];				/*OEM RTCM task*/
OS_STK RTK_DATE_TASK_STK[RTK_DATE_STK_SIZE];				/*RTK DATE task*/
OS_STK ESP_DATE_TASK_STK[ESP_DATE_STK_SIZE];				/*ESP DATE task*/
OS_STK RTCM_TASK_STK[RTCM_STK_SIZE];						/*RTCM DATE task*/
/*****************************************************/

int main(void)
{
	SystemInit(); 
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4);//
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);		//	1ms base time	
	SOUNCE_Init();
/***************USB***********************/
	#ifdef USE_USB_OTG_FS
	
	  USBD_Init(&USB_OTG_dev,
	#ifdef USE_USB_OTG_HS 
            USB_OTG_HS_CORE_ID,
	#else            
            USB_OTG_FS_CORE_ID,
	#endif  
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
	
	#endif  
/********************************  */
	LED_Init();
	fm25v_init();
	MPU9250_Init();
	Key_Init();
	//RTK_BASE_Init();
	
/***********OS setup****************/
	OSInit(); 
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();
/**********************************/

	while(1)
	{
		OSTimeDlyHMSM(0,0,1,0);
		//printf("ok\r\n");
	}
}

void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr = 0;
  pdata = pdata;
  OS_ENTER_CRITICAL();            //进入临界区(无法被中断打断)

	OSTaskCreate(tickit_task, (void *)0, (OS_STK*)&TICKIT_TASK_STK[TICKIT_STK_SIZE - 1], TICKIT_TASK_PRIO);
	OSTaskCreate(led_task, (void *)0, (OS_STK*)&LED_TASK_STK[LED_STK_SIZE - 1], LED_TASK_PRIO);
	OSTaskCreate(key_task, (void *)0, (OS_STK*)&KEY_TASK_STK[KEY_STK_SIZE - 1], KEY_TASK_PRIO);
	OSTaskCreate(oem_rtcm_task, (void *)0, (OS_STK*)&OEM_RTCM_TASK_STK[OEM_RTCM_STK_SIZE - 1], OEM_RTCM_TASK_PRIO);
	OSTaskCreate(rtk_date_task, (void *)0, (OS_STK*)&RTK_DATE_TASK_STK[RTK_DATE_STK_SIZE - 1], RTK_DATE_TASK_PRIO);
	OSTaskCreate(esp_date_task, (void *)0, (OS_STK*)&ESP_DATE_TASK_STK[ESP_DATE_STK_SIZE - 1], ESP_DATE_TASK_PRIO);
	OSTaskCreate(rtcm_task, (void *)0, (OS_STK*)&RTCM_TASK_STK[RTCM_STK_SIZE - 1], RTCM_TASK_PRIO);
	
	OEM_RTCM_Semp=OSSemCreate(0);
	RTK_DATE_Semp=OSSemCreate(0);
	ESP_DATE_Semp=OSSemCreate(0);

	OSTaskSuspend(RTCM_TASK_PRIO); //挂起起始任务.
	OSTaskSuspend(START_TASK_PRIO); //挂起起始任务.
  OS_EXIT_CRITICAL();             //退出临界区(可以被中断打断)
}

void tickit_task(void *pdata)
{
	while(1)
	{
		OSTimeDlyHMSM(0,0,1,0);
		wifi_tick_time();
		RTK_tick_time();
		
		timer_tick(1);
		restart_connect();	
	}
}

uint8_t buff[18]="OK-ZAM-HELLO-FEIMA";
void led_task(void *pdata)
{
 	static uint32_t dt;
	static uint8_t buffa[18];
	while(1)
	{
		OSTimeDlyHMSM(0,0,1,0);
		LED_Out(POW_GPIO1,ON_OFF);
		//wifi_soc_send(0,"ok send date",12);
		//UART_SendString(ESP_PER,"AT\r\n");
		//dt=micros();
		//Get_Raw_Date();
		//printf("get_time=%d\r\n",dt);
		Fram_read(0x0000,18,&buffa[0]);
	}	
}

void key_task(void *pdata)
{
	static unsigned char key_sttus=0;
	while(1)
	{
		OSTimeDlyHMSM(0,0,0,10);
		key_sttus=Key_Scan();
	}	
}
void oem_rtcm_task(void *pdata)
{
	INT8U err;
	static unsigned int crc_value_rtcm=0;
	static unsigned char rtcm_cnt=0;
	while(1)
	{
		OSSemPend(OEM_RTCM_Semp,0,&err);
		for(rtcm_cnt=0;rtcm_cnt<7;rtcm_cnt++)
		{
			if(rtcm_msg[rtcm_cnt].rtcm_flag==1)
			{
				crc_value_rtcm=CRC_Octets(&rtcm_msg[rtcm_cnt].head,(rtcm_msg[rtcm_cnt].lenth_h<<8)+rtcm_msg[rtcm_cnt].lenth_l+3);
				if(crc_value_rtcm==((rtcm_msg[rtcm_cnt].rtcm_buff[(rtcm_msg[rtcm_cnt].lenth_h<<8)+rtcm_msg[rtcm_cnt].lenth_l]<<16)+
				(rtcm_msg[rtcm_cnt].rtcm_buff[(rtcm_msg[rtcm_cnt].lenth_h<<8)+rtcm_msg[rtcm_cnt].lenth_l+1]<<8)+
				(rtcm_msg[rtcm_cnt].rtcm_buff[(rtcm_msg[rtcm_cnt].lenth_h<<8)+rtcm_msg[rtcm_cnt].lenth_l+2])))
				{
					USARTx_SendBytes(RTK_RTCM,&rtcm_msg[rtcm_cnt].head,rtcm_msg[rtcm_cnt].lenth_h*256+rtcm_msg[rtcm_cnt].lenth_l+6);
				}
				rtcm_msg[rtcm_cnt].rtcm_flag=0;
			}
		}
	}
}
void rtk_date_task(void *pdata)
{
	INT8U err;
	RTK_BASE_Init();
	OSTimeDlyHMSM(0,0,2,0);//wait 2s
	while(OEM_Init_output());
	while(1)
	{
		OSSemPend(RTK_DATE_Semp,0,&err);
		if(rtk_date_mesg.rtk_date_flag==1)
		{
			wifi_soc_send(0,&rtk_date_mesg.rtcm_buff[0],rtk_date_mesg.rtk_date_lenth);
			//USARTx_SendBytes(RTK_DAT,&rtk_date_mesg.rtcm_buff[0],rtk_date_mesg.rtk_date_lenth);
			memset(&rtk_date_mesg.rtk_date_flag,0,sizeof(RTK_DATE_MESG));
		}
	}
}

void esp_date_task(void *pdata)
{
	INT8U err;
	static unsigned char wifi_chan=0;
	Wifi_Esp_Init();
	while(wifi_reg());
	LED_Out(POW_GPIO2,ON_OFF);
	while(1)
	{
		OSSemPend(ESP_DATE_Semp,0,&err);
		if(esp_mesg.esp__date_flag==1)
		{
			for(wifi_chan=0;wifi_chan<WIF_CHAN_CNT;wifi_chan++)
			{
				if(WIFI_CHAN_ON_FLAG[wifi_chan]==1)
				{
					if(strstr(&esp_mesg.esp_date_buff[0],"rtkconfig") != NULL)
					{
						OEM_RTK_BASE();
					}
					USARTx_SendBytes(RTK_DAT,&esp_mesg.esp_date_buff[0],esp_mesg.esp_date_lenth);
					//wifi_soc_send(wifi_chan,&esp_mesg.esp_date_buff[0],esp_mesg.esp_date_lenth);
				}
			}
			Clean_ESP_date();
		}
	}
}

void rtcm_task(void *pdata) 
{
	static uint32_t qxwz_tick_cnt = 0;
	static int curretnTime = 0;
	qxwz_soc_close(0);
	qxwz_soc_close(1);
  while(1) 
	{
			OSTimeDlyHMSM(0,0,1,0);//wait 1s每隔一秒模拟宿主触发一次tick
	   	curretnTime += 1000;
			qxwz_tick_cnt++;
			async_notify();
    	qxwz_tick(curretnTime);
		//get_qxwz_sdk_account_info();
  }
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