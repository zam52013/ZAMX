/****************************************************************************
 *@file mou9250.h
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
 #include "mpu9250.h"
 unsigned char Mpu9250_Read_Reg(unsigned char reg)
{
	unsigned char value;
	MPU9250_X_select(MPU9250_CS,MPU9250_ID,CS_ON);
	SPI_ReadWriteByte(MPU9250,reg|DIR_READ);
	value=SPI_ReadWriteByte(MPU9250,DIR_WRITE);
	MPU9250_X_select(MPU9250_CS,MPU9250_ID,CS_OFF);
	return value;
}
unsigned char Mpu9250_Write_Reg(unsigned char reg,unsigned char value)
{
	unsigned char status;
	MPU9250_X_select(MPU9250_CS,MPU9250_ID,CS_ON);
	status=SPI_ReadWriteByte(MPU9250,reg);
	SPI_ReadWriteByte(MPU9250,value);
	MPU9250_X_select(MPU9250_CS,MPU9250_ID,CS_OFF);
	return status;
}
 void MPU9250_Init()
 {
		SPIx_Init(MPU9250);
		SPIx_Init_Cs(MPU9250_CS);
	 
		Mpu9250_Write_Reg(MPUREG_PWR_MGMT_1, 0x03);//pow config
		Mpu9250_Write_Reg(MPUREG_PWR_MGMT_2, 0x00);//pow config
    Mpu9250_Write_Reg(MPUREG_SMPLRT_DIV, 0x00);//samp speed
    Mpu9250_Write_Reg(MPUREG_CONFIG, 0x02);
    Mpu9250_Write_Reg(MPUREG_GYRO_CONFIG, 0x08);//500
    Mpu9250_Write_Reg(MPUREG_ACCEL_CONFIG, 0x08);// 4g
    Mpu9250_Write_Reg(MPUREG_ACCEL_CONFIG2, 0x0E);
 }
 u8 raw_dat[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 void Get_Raw_Date(void)
{
	unsigned char i;
	for(i=0;i<14;i++)
	{
		raw_dat[i]=Mpu9250_Read_Reg(MPUREG_ACCEL_XOUT_H+i);
	}
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
 