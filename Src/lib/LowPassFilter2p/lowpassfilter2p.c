/****************************************************************************
 *@file lowpassfilter2p.c
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
 
 #include <math.h>
#include "lowpassfilter2p.h"

lowpassfilter2p Set_Cutoff_Frequency(float samp_freq,float cutoff_freq)
{
	lowpassfilter2p lowpassfilter_date;
	float fr;
	float ohm;
	float c;
	lowpassfilter_date._sample_freq=samp_freq;
	lowpassfilter_date._cutoff_freq=cutoff_freq;
	fr=samp_freq/cutoff_freq;
	ohm=tanf(_M_PI_F/fr);
	c=1.0+2.0*cosf(_M_PI_F/4.0f)*ohm+ohm*ohm;
	lowpassfilter_date._b0=ohm*ohm/c;
	lowpassfilter_date._b1=2.0f*lowpassfilter_date._b0;
	lowpassfilter_date._b2=lowpassfilter_date._b0;
	lowpassfilter_date._a1=2.0f*(ohm*ohm-1.0f)/c;
	lowpassfilter_date._a2=(1.0f-2.0f*cosf(_M_PI_F/4.0f)*ohm+ohm*ohm)/c;
	return lowpassfilter_date;
}

float LowPassFilter2p_Apply(lowpassfilter2p lowpassfilter_date,float samp_value)
{
	float delay_element_0;
	float output;
	if(lowpassfilter_date._cutoff_freq<=0.0f)
	{
		return samp_value;
	}
	delay_element_0=samp_value-lowpassfilter_date._delay_element_1*lowpassfilter_date._a1-lowpassfilter_date._delay_element_2*lowpassfilter_date._a2;
	output=delay_element_0*lowpassfilter_date._b0+lowpassfilter_date._delay_element_1*lowpassfilter_date._b1+lowpassfilter_date._delay_element_2*lowpassfilter_date._b2;
	lowpassfilter_date._delay_element_2=lowpassfilter_date._delay_element_1;
	lowpassfilter_date._delay_element_1=delay_element_0;
	return output;
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
