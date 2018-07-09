/****************************************************************************
 *@file imu.c
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
 * @date 2018-07-09
 * 
 * 
 ****************************************************************************/
#include "imu.h"
#include <math.h>

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 
float exInt = 0, eyInt = 0, ezInt = 0;        // 
float gx=0,gy=0,gz=0,ax=0,ay=0,az=0;          //  



float Yaw,Pitch,Rool;  //偏航角，俯仰，横滚

//ang= kfa*g+kfgn*acc;

//====================================================================================================
// Function
//====================================================================================================
//gx 采样时间增量 

//四元素输出

void IMUupdate(float gxi, float gyi, float gzi, float axi, float ayi, float azi) {
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;  
        //增加互补滤波
                float q0q0 = q0*q0;
                float q0q1 = q0*q1;
                float q0q2 = q0*q2;
                float q0q3 = q0*q3;
                float q1q1 = q1*q1;
                float q1q2 = q1*q2;
                float q1q3 = q1*q3;
                float q2q2 = q2*q2;
                float q2q3 = q2*q3;
                float q3q3 = q3*q3;

        ax=ax*kfa+kfan*axi;
        ay=ay*kfa+kfan*ayi;
        az=az*kfa+kfan*azi;

        gx=gx*kfg+kfgn*gxi;
        gy=gy*kfg+kfgn*gyi;
        gz=gz*kfg+kfgn*gzi;

        // 测量量化
        norm = sqrt(ax*ax + ay*ay + az*az); 
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;      
        
        // 估计重力方向
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
        
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
        
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
        
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
        Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
        Rool = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
}