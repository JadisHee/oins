#ifndef AHRS_h
#define AHRS_h

#include <math.h>

class AHRS
{
private:
    static float invSqrt(float x);
    float beta;
    float q0;
    float q1;
    float q2;
    float q3;

    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    
    char anglesComputed;

    void computeAngles();
    
    
    /* data */
public:
    AHRS(void);
    // ~AHRS(static float invSqrt(fl;oat x)
    void begin(float sampleFrequency)
    {
        invSampleFreq = 1.0f / sampleFrequency;
    }
    void update(float gx, float gy, float gz, float ax, float ay, float az);

    float getRoll()
    {
        if(!anglesComputed) computeAngles();
        return roll * 180 / M_PI;
    }

    float getpitch()
    {
        if(!anglesComputed) computeAngles();
        return pitch * 180 / M_PI;
    }
    
    float getyaw()
    {
        if(!anglesComputed) computeAngles();
        return yaw * 180 / M_PI;
    }

    float getRollRadians()
    {
        if(!anglesComputed) computeAngles();
        return roll;
    }

    float getPitchRadians()
    {
        if(!anglesComputed) computeAngles();
        return pitch;
    }

    float getYawRadians()
    {
        if(!anglesComputed) computeAngles();
        return yaw;
    }

};
#endif