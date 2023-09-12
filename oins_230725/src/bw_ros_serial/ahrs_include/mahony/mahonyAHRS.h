#ifndef mahonyAHRS_h
#define mahonyAHRS_h
#include <math.h>

class mahony
{
    private:
        // 两倍的比例系数
        float twoKp;
        // 两倍的积分系数
        float twoKi;
        // 四元数
        float q0,q1,q2,q3;
        // 以Ki为尺度的积分误差项
        float integralFBx, integralFBy, integralFBz;
        // 设置样本数据频率的倒数
        float invSampleFreq;
        // 欧拉角
        float roll, pitch, yaw;
        
        char anglesComputed;
        
        static float invSqrt(float x);
	    
        void computeAngles();

    public:
        
        mahony();
        
        void begin(float sampleFrequency)
        {
            invSampleFreq = 1.0f / sampleFrequency;
        }

        void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    
        float getRoll()
        {
            if(!anglesComputed)
                computeAngles();
            return roll * 57.29578f;
        }

        float getPitch()
        {
            if(!anglesComputed)
                computeAngles();
            return pitch * 57.29578f;
        }

        float getYaw()
        {
            if(!anglesComputed)
                computeAngles();
            return yaw * 57.29578f;
        }
};

#endif