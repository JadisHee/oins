#include "mahonyAHRS.h"
#include <math.h>

// 定义数据频率
#define DEFAULT_SAMPLE_FREQ 512.0f

// 定义kp
#define twoKpDef (2.0f * 5.0f)
// 定义ki
#define twoKiDef (2.0f * 0.0f)


mahony::mahony()
{
    twoKp = twoKpDef;
    twoKi = twoKiDef;
    
    // 初始化各项参数
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    anglesComputed = 0;
    invSampleFreq = 1.0f / DEFAULT_SAMPLE_FREQ;
}

void mahony::updateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 将角度制转换为弧度制，deg/s --->  rad/s
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // 将加速度数据进行归一化
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 从四元数提出的重力方向的预测分量
        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 + q2*q3;
        halfvz = q0*q0 - 0.5f + q3*q3;

        /*
        halfex   ax   halfvx
        halfey = ay x halfvy  
        halfez   az   halfbz
        */

        // 计算测量加速度方向与预测重力分量的叉乘 
        halfex = (ay*halfvz - az*halfvy);
        halfey = (az*halfvx - ax*halfvz);
        halfez = (ax*halfvy - ay*halfvx);

        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * invSampleFreq;
            integralFBy += twoKi * halfey * invSampleFreq;
            integralFBz += twoKi * halfez * invSampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    gx *= (0.5f * invSampleFreq);
    gy *= (0.5f * invSampleFreq);
    gz *= (0.5f * invSampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    anglesComputed = 0;
}

float mahony::invSqrt(float x)
{
    float halfx = 0.5f * x;
	
    union 
    {
        float f; 
        long l; 
    }
    i;
	
    i.f = x;
	i.l = 0x5f3759df - (i.l >> 1);
	float y = i.f;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void mahony::computeAngles()
{
    roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	anglesComputed = 1;
}