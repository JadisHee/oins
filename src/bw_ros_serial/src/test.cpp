#include "../ahrs_include/mahony/mahonyAHRS.h"
#include <ros/ros.h>
#include <opencv/cv.h>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include "serial/serial.h"
#include <math.h>
#include <chrono>
#include <thread>

#include <fstream>

#include <sensor_msgs/Imu.h>

// 设置串口参数
serial::Serial imu_ser("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(100));

/**
 * @brief 用于解算imu回传加速度数据
 * @param data 回传报文
 * @param start_bit 加速度数据的起始位
 * @returns a 某轴加速度
*/
double read_acc(std::string data, int start_bit)
{
    char dot = '.';
    char a_1 = data[start_bit];
    char a_2 = data[start_bit + 1];
    char a_3[4];
    for (int i = 0; i < 4; i++)
    {
        a_3[i] = data[i + start_bit + 2];
    }
    bool a_1_ = a_1 - '0';
    int a_2_ = a_2 - '0';
    int a_3_ = a_3[0] - '0';
    int a_4_ = a_3[1] - '0';
    int a_5_ = a_3[2] - '0';
    int a_6_ = a_3[3] - '0';
    int decimal = dot == '.' ? 1 : 0;
    double a;
    if (a_1_)
    {
        a = (a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal));
    }
    else
    {
        a = -(a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal));
    }

    // float a = (a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal));

    return a;
}

/**
 * @brief 用于解算imu回传角度/角速度数据
 * @param data 回传报文
 * @param start_bit 角度/角速度数据的起始位
 * @returns g 某轴角度/角速度
*/
double read_gyr(std::string data, int start_bit)
{
    char dot = '.';
    char g_1 = data[start_bit];
    char g_2[3];
    for (int i = 0; i < 3; i++)
    {
        g_2[i] = data[i + start_bit + 1];
    }
    char g_3[2];
    for (int i = 0; i < 2; i++)
    {
        g_3[i] = data[i + start_bit + 4];
    }
    bool g_1_ = g_1 - '0';
    int g_2_ = g_2[0] - '0';
    int g_3_ = g_2[1] - '0';
    int g_4_ = g_2[2] - '0';

    int g_5_ = g_3[0] - '0';
    int g_6_ = g_3[1] - '0';

    int decimal = dot == '.' ? 1 : 0;
    double g;
    if (g_1_)
    {
        g = (g_2_ * 100 + g_3_ * 10 + g_4_ + g_5_ * pow(10, -1 * decimal) + g_6_ * pow(100, -1 * decimal));
    }
    else
    {
        g = -(g_2_ * 100 + g_3_ * 10 + g_4_ + g_5_ * pow(10, -1 * decimal) + g_6_ * pow(100, -1 * decimal));
    }

    // float a = (a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal));

    return g;

}

/**
 * @brief 用于计算一个数组的平均值
 * @param 被求数组
 * @return 被求数组的平均值
*/
double average(double *x)
{
    int length = sizeof(x)/sizeof(*x);

    double sum = 0;
    for(int i = 0;i < length; i++)
    {
        sum += x[i];
    }
    return sum/length;
}

/**
 * @brief 用于通过姿态解算出由机体系转换至地平坐标系下的姿态矩阵
 * @param r 滚转角
 * @param p 俯仰角
 * @param y 偏航角
 * @return 姿态矩阵
*/
cv::Mat C_bn(double r, double p, double y)
{
    cv::Mat cbn = cv::Mat::ones(3,3,CV_32FC1);
    cbn.at<float>(0,0) = cos(p)*cos(y);
    cbn.at<float>(0,1) = sin(r)*sin(p)*cos(y)-cos(r)*sin(y);
    cbn.at<float>(0,2) = cos(r)*sin(p)*cos(y)+sin(r)*sin(y);
    
    cbn.at<float>(1,0) = cos(p)*sin(y);
    cbn.at<float>(1,1) = sin(r)*sin(p)*sin(y)+cos(r)*cos(y);
    cbn.at<float>(1,2) = cos(r)*sin(p)*sin(y)-sin(r)*cos(y);

    cbn.at<float>(2,0) = -sin(p);
    cbn.at<float>(2,1) = sin(r)*cos(p);
    cbn.at<float>(2,2) = cos(r)*cos(p);

    return cbn;
}

using namespace std;


int main(int argc, char **argv)
{
    ofstream outfile("/home/jadis/work/oins/src/bw_ros_serial/data/230625/record_3.csv");

    outfile << "dt,acc_x,acc_y,"
                << "gyr_x,gyr_y,"
                << "r_imu,p_imu,y_imu,"
                << "r_cal,p_cal,y_cal,"
                << "ax_n_imu,ay_n_imu,"
                << "ax_n_cal,ay_n_cal,"
                << "ax_n_cal_kalman,dax_n_cal_kalman,"
                << "dax_n_imu,dax_n_cal\n";

    mahony imu50;

    // 待发送的报文
    uint8_t imu_send_data[] = {0x77, 0x04, 0x00, 0x59, 0x5D};
    int imu_send_size = sizeof(imu_send_data) / sizeof(uint8_t);

    // 接收回传报文
    std::string imu_response;

    // 定义变换矩阵
    cv::Mat cbn_imu;
    cv::Mat cbn_cal;

    cout << "等待AGV开始运动!" << endl;
    sleep(1);


    int agv_mod = 0;
    // 声明比力
    cv::Mat acc = cv::Mat::ones(3,1,CV_32FC1);
    
    // 声明角速度
    cv::Mat gyr = cv::Mat::ones(3,1,CV_32FC1);

    // 声明在地平导航坐标系下的运动加速度
    cv::Mat acc_n_imu;
    cv::Mat acc_n_cal;

    // 上一时刻的时间
    auto time_t_0 = chrono::steady_clock::now();
    
    // 程序开始时候的时间
    auto time_start = chrono::steady_clock::now();
    
    // 将位移和速度初始化为零
    float displacement_x = 0.000;
    float displacement_y = 0.000;

    float velocity_x = 0.000;
    float velocity_y = 0.000;
    
    // 声明读取的欧拉角
    double roll_imu,pitch_imu,yaw_imu;
    
    double roll_cal,pitch_cal,yaw_cal;

    // 声明由读取欧拉角算出来的地平加速度
    float ax_n_imu;
    float ay_n_imu;

    // 声明由解算欧拉角算出来的地平加速度
    float ax_n_cal;
    float ay_n_cal;
    // ----------------------------------------------------------------
    // ----------------------------卡尔曼变量----------------------------
    // ----------------------------------------------------------------
    
    // 预测优化的初始状态
    cv::Mat X = cv::Mat::zeros(2,1,CV_32FC1);

    // 状态转移矩阵
    cv::Mat A = cv::Mat::ones(2,2,CV_32FC1);
    A.at<float>(0,1) = 0.01;
    A.at<float>(1,0) = 0.0;

    // 初始状态协方差矩阵
    cv::Mat P = cv::Mat::ones(2,2,CV_32FC1);
    P.at<float>(0,1) = 0.0;
    P.at<float>(1,0) = 0.0;

    // 状态转移(预测噪声)协方差矩阵
    cv::Mat Q = cv::Mat::zeros(2,2,CV_32FC1);
    Q.at<float>(0,0) = 0.005;
    Q.at<float>(1,1) = 0.005;

    // 观测矩阵
    cv::Mat H = cv::Mat::zeros(1,2,CV_32FC1);
    H.at<float>(0,0) = 1.0;

    // 观测噪声协方差
    float R = 1.0;
    
    // 声明读取欧拉角x轴地平加速度卡尔曼滤波后的结果
    // float ax_n_cal_kalman;

    // 声明解算欧拉角x轴地平加速度卡尔曼滤波后的结果
    float ax_n_cal_kalman;

    // ----------------------------------------------------------------
    // ----------------------------开始循环-----------------------------
    // ----------------------------------------------------------------
    while(true)
    {
        // 计算两次解算之间的时间差（周期）
        auto time_now = chrono::steady_clock::now();
        auto duration = chrono::duration_cast<std::chrono::milliseconds>(time_now - time_t_0);
        double time_dt = duration.count();
        time_t_0 = time_now;
        float dt = time_dt/1000;


        // ----------------------------报文收发----------------------------
        // 将待发送的报文发送至串口
        imu_ser.write(imu_send_data, imu_send_size);

        // 接收回传报文
        vector<uint8_t> imu_recv_data;

        // 读取串口返回的数据，大小为48个bytes
        imu_ser.read(imu_recv_data, 48);

        // 如果接收到的数据不足48bytes，则跳过此次循环
        if (imu_recv_data.size() != 48)
        {
            continue;
        }

        // 将回传数据转为字符串
        imu_response.resize(48*2);
        static constexpr char hex_digits[] = "0123456789ABCDEF";
        for (std::size_t i = 0; i < 48; ++i)
        {
            imu_response[i * 2] = hex_digits[imu_recv_data[i] >> 4];
            imu_response[i * 2 + 1] = hex_digits[imu_recv_data[i] & 0xF];
        }

        // 读取比力
        acc.at<float>(0,0) = read_acc(imu_response, 26) * 9.79134;
        acc.at<float>(1,0) = read_acc(imu_response, 32) * 9.79134;
        acc.at<float>(2,0) = read_acc(imu_response, 38) * 9.79134;

        // 读取角速度
        gyr.at<float>(0,0) = - read_gyr(imu_response, 50);
        gyr.at<float>(1,0) =   read_gyr(imu_response, 44);
        gyr.at<float>(2,0) = - read_gyr(imu_response, 56);

        // ------------------------------------------------------------
        // ------------------------------读取欧拉角----------------------
        // ------------------------------------------------------------
        // 声明上一时刻的读取欧拉角
        double roll_imu_t = roll_imu;
        double pitch_imu_t = pitch_imu;

        // 读取角度
        roll_imu = -read_gyr(imu_response, 14);
        pitch_imu = read_gyr(imu_response, 8);
        yaw_imu = 0;

        // 计算两组数据间的差值
        double dr = abs(roll_imu - roll_imu_t);
        double dp = abs(pitch_imu - pitch_imu_t);

        // ------------------------------------------------------------
        // ------------------------------计算欧拉角----------------------
        // ------------------------------------------------------------
        // 声明上一时刻的计算欧拉角
        double roll_cal_t = roll_cal;
        double pitch_cal_t = pitch_cal;

        // 代入互补滤波解算欧拉角
        imu50.begin(1/dt);
        imu50.updateIMU(gyr.at<float>(0,0),gyr.at<float>(1,0),gyr.at<float>(2,0),acc.at<float>(0,0),acc.at<float>(1,0),acc.at<float>(2,0));

        roll_cal = imu50.getRoll();
        pitch_cal = imu50.getPitch();
        yaw_cal = 0;
        
        // 计算两组数据间的差值
        double dr_cal = abs(roll_cal - roll_cal_t);
        double dp_cal = abs(pitch_cal - pitch_cal_t);

        // ------------------------------------------------------------
                
        cout << "此时的读取欧拉角为：\n" << " roll_imu: \n  " <<  roll_imu << "\n pitch_imu:\n  " << pitch_imu << "\n yaw_imu:\n  " << yaw_imu  << endl;

        cout << "此时的计算欧拉角为：\n" << " roll_cal: \n  " <<  roll_cal << "\n pitch_cal:\n  " << pitch_cal << "\n yaw_cal:\n  " << yaw_cal  << endl;

        // 分别算出二者所得出的旋转矩阵
        cbn_imu = C_bn(roll_imu * M_PI / 180,pitch_imu * M_PI / 180,yaw_imu * M_PI / 180);

        cbn_cal = C_bn(roll_cal * M_PI / 180,pitch_cal * M_PI / 180,yaw_cal * M_PI / 180);
        
        // ------------------------------------------------------------
        // --------------------将比力变换之导航坐标系下--------------------
        // ------------------------------------------------------------
        acc_n_imu = cbn_imu * acc;
        float ax_n_imu_t = ax_n_imu;
        float ay_n_imu_t = ay_n_imu;

        ax_n_imu = -acc_n_imu.at<float>(0,0);
        ay_n_imu = -acc_n_imu.at<float>(1,0);

        double dax_n_imu = ax_n_imu - ax_n_imu_t;
        double day_n_imu = ay_n_imu - ay_n_imu_t;

        // ------------------------------
        acc_n_cal = cbn_cal * acc;
        float ax_n_cal_t = ax_n_cal;
        float ay_n_cal_t = ay_n_cal;

        ax_n_cal = -acc_n_cal.at<float>(0,0);
        ay_n_cal = -acc_n_cal.at<float>(1,0);

        double dax_n_cal = ax_n_cal - ax_n_cal_t;
        double day_n_cal = ay_n_cal - ay_n_cal_t;        

        // ----------------------------------------------------------------
        // ----------------------------卡尔曼过程----------------------------
        // ----------------------------------------------------------------

        cv::Mat X_predict = A * X;
        cv::Mat P_predict = A * P * A.t() + Q;

        cv::Mat K = P_predict * H.t() * (1 / (H * P_predict * H.t() + R));
        X = X_predict + K * (ax_n_cal - H * X_predict);
        P = (cv::Mat::eye(2,2,CV_32FC1) - K * H) * P_predict;

        float ax_n_cal_kalman_t = ax_n_cal_kalman;
        
        ax_n_cal_kalman = X.at<float>(0,0);

        double dax_n_cal_kalman = ax_n_cal_kalman - ax_n_cal_kalman_t;
        // ----------------------------------------------------------------



        // ------------------------------------------------------------

        float ax_b = acc.at<float>(0,0);
        float ay_b = acc.at<float>(1,0);

        outfile << dt << "," << acc.at<float>(0,0) << "," << acc.at<float>(1,0) << "," 
                << gyr.at<float>(0,0) << "," << gyr.at<float>(1,0) << "," 
                << roll_imu << "," << pitch_imu << "," << yaw_imu << "," 
                << roll_cal << "," << pitch_cal << "," << yaw_cal << ","
                << ax_n_imu << "," << ay_n_imu << ","
                << ax_n_cal << "," << ay_n_cal << ","
                << ax_n_cal_kalman << "," << abs(dax_n_cal_kalman) << ","
                << abs(dax_n_imu) << "," << abs(dax_n_cal) << "\n";
        cout << "机体坐标系下比力为:\n" << " x轴:\n  " << ax_b << "\n y轴:\n  " << ay_b << endl;
        cout << "地平坐标系下读取欧拉角算出的加速度为:\n" << " x轴:\n  " << ax_n_imu << "\n y轴:\n  " << ay_n_imu << endl;
        cout << "地平坐标系下解算欧拉角算出的加速度为:\n" << " x轴:\n  " << ax_n_cal << "\n y轴:\n  " << ay_n_cal << endl;

        cout << "与上一时刻的读取加速度差为:\n" << " x:\n  " << dax_n_imu << "\n y:\n  " << day_n_imu << endl;
        cout << "与上一时刻的解算加速度差为:\n" << " x:\n  " << dax_n_cal << "\n y:\n  " << day_n_imu << endl;

        cout << "---" << endl;

    }

    outfile.close();
    return 0;
}
