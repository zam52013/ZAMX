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
OS_STK LED_TASK_STK[LED_STK_SIZE];				/*LED task*/
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
	
/***********OS setup****************/
	OSInit(); 
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();
/**********************************/

	while(1)
	{
		//printf("ok\r\n");
	}
}

void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr = 0;
  pdata = pdata;
  OS_ENTER_CRITICAL();            //进入临界区(无法被中断打断)

	OSTaskCreate(led_task, (void *)0, (OS_STK*)&LED_TASK_STK[LED_STK_SIZE - 1], LED_TASK_PRIO);

	OSTaskSuspend(START_TASK_PRIO); //挂起起始任务.
  OS_EXIT_CRITICAL();             //退出临界区(可以被中断打断)
}
    
void led_task(void *pdata)
{
	while(1)
	{
		OSTimeDlyHMSM(0,0,0,1000);
		LED_Out(POW_GPIO1,ON_OFF);
		printf("get_time=%d\r\n",OSTimeGet());
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