/****************************************************************************
 *@file key.c
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
 * @date 2018-07-05
 * 
 * 
 ****************************************************************************/
 
 #include "key.h"
 
 void Key_Init()
 {
	GPIOx_Init(KEY_GPIO);
 }
 
 unsigned char Key_Scan(void)
{
	static unsigned char key_stus=0;
	 static unsigned int KEY_COUNT=0;
	unsigned char KEY_FUN;
	unsigned key_buff=GPIOx_Read_Input(KEY_GPIO);
	
	switch(key_stus)
	{
		case 0:	
			if(key_buff==0)//DOWN
			{
				key_stus++;
				KEY_FUN=KEY_FILE;//next stat
			}
			else
			{
				key_stus=0;
			}
			break;
		case 1:
			if(key_buff==0)
			{
				key_stus++;
				KEY_FUN=KEY_SHORT_DOWN;//down
				KEY_COUNT=0;//clean count
			}
			else
			{
				key_stus=0;//erro
			}
			break;
		case 2:
			if(key_buff==0)
			{
				KEY_COUNT++;
				if(KEY_COUNT>KEY_LONG_TIME)//long key 
				{
					key_stus++;
					KEY_COUNT=KEY_LONG_TIME+1;
					KEY_FUN=KEY_LONG_DOWN;//long key
				}
			}
			else
			{
				key_stus=0;
				if(KEY_COUNT>KEY_SHORT_TIME)
				{
					KEY_FUN=KEY_SHORT_UP;//short up
				}
				else
				{
					KEY_FUN=KEY_NULL;
				}
			}
			break;
		case 3:
			if(key_buff)
			{
				KEY_FUN=KEY_LONG_UP;//key up
				key_stus=0;
			}
			break;
		default:
			key_stus=0;
	}
	return KEY_FUN;
}

	/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT feima *****END OF FILE****/

 