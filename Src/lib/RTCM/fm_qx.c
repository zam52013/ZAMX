/****************************************************************************
 *@file fm_qx.c
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
 * @date 2018-07-06
 * 
 * 
 ****************************************************************************/
 #include "fm_qx.h"

/*******************************************************************************
函 数 名：void qxwz_status_response_callback(void)
功能描述： 查询用户账号信息
入口参数：							
返回参数：账户信息
创建时间: 2017-11-02 by zam
********************************************************************************/
void qxwz_status_response_callback(qxwz_rtcm_status_code status)
{
	
	printf("QXWZ_STATUS =%d",status);
	switch(status)
	{
		case QXWZ_NET_WORK_ERROR:
			//UART_SendString(BLUETOOTH_USART,"$QXWZ_NET_WORK_ERROR\r\n");
			break;
		case QXWZ_APPKEY_IDENTIFY_FAIL:
			//UART_SendString(BLUETOOTH_USART,"$QXWZ_APPKEY_IDENTIFY_FAIL\r\n");
			break;
		case QXWZ_DEVICEID_NULL:
			//UART_SendString(BLUETOOTH_USART,"$QXWZ_DEVICEID_NULL\r\n");
			break;
		case QXWZ_STATUS_OPENAPI_ACCOUNT_NOT_EXIST:
			//UART_SendString(BLUETOOTH_USART,"$QXWZ_STATUS_OPENAPI_ACCOUNT_NOT_EXIST\r\n");
			break;
		case QXWZ_STATUS_OPENAPI_DUPLICATE_ACCOUNT:
			//UART_SendString(BLUETOOTH_USART,"$QXWZ_STATUS_OPENAPI_DUPLICATE_ACCOUNT\r\n");
			break;
		case QXWZ_STATUS_OPENAPI_DISABLED_ACCOUNT:
			//UART_SendString(BLUETOOTH_USART,"$QXWZ_STATUS_OPENAPI_DISABLED_ACCOUNT\r\n");
			break;
		case QXWZ_STATUS_OPENAPI_ACCOUNT_EXPIRED:
			//UART_SendString(BLUETOOTH_USART,"$QXWZ_STATUS_OPENAPI_ACCOUNT_EXPIRED\r\n");
			break;
		case QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE:
			//UART_SendString(BLUETOOTH_USART,"$QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE\r\n");
			break;
		case QXWZ_STATUS_NTRIP_UNAUTHORIZED:
			//UART_SendString(BLUETOOTH_USART,"$QXWZ_STATUS_NTRIP_UNAUTHORIZED\r\n");
			break;
		default:
			break;
	}
	if(status == QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE)
	{
		printf("expire_time=%d\n",getqxwzAccount()->expire_time);
	}
}
/*******************************************************************************
函 数 名：void get_qxwz_sdk_account_info(void)
功能描述： 查询用户账号状态
入口参数：							
返回参数：
创建时间: 2017-11-02 by zam
********************************************************************************/
void  get_qxwz_sdk_account_info(void)
{
	qxwz_account_info *p_account_info = NULL;
	p_account_info = getqxwzAccount();
	if(p_account_info->appkey != NULL) 
	{
		printf("appkey=%s\n",p_account_info->appkey);
	}
	if(p_account_info->NtripPassword != NULL) 
	{
		printf("NtripPassword=%s\n",p_account_info->NtripPassword);
	}
}
/*******************************************************************************
函 数 名：void qxwz_rtcm_response_callback(void)
功能描述： 打印差分数据
入口参数：							
返回参数：
创建时间: 2017-11-02 by zam
********************************************************************************/
void qxwz_rtcm_response_callback(char* rtcm, size_t length)
{
	uint16_t i=0;
	unsigned int total_recv_bytes = 0;
	total_recv_bytes += length;
	for(;i<length;i++)
	{
		//printf("%02x",rtcm[i]);
		//printf(" ");
	//	while(USART_GetFlagStatus(RTCM_USART, USART_FLAG_TC)==RESET); 
	//	USART_SendData(RTCM_USART ,rtcm[i]);//发送当前字符
	}
	//printf("total_recv_bytes = %d\n",total_recv_bytes);
	//printf("\n");
}
