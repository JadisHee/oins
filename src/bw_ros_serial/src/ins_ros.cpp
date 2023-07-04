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
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry ins_odom;

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
double read_gyo(std::string data, int start_bit)
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
    ros::init(argc, argv, "ins_ros_node");
    
    ros::NodeHandle nh;

    ros::Publisher ins_pub = nh.advertise<nav_msgs::Odometry>("/ins/odom",100);



    // --------------------------------------------------------
    // ----------------------------初始化标定过程----------------
    // --------------------------------------------------------
    cout << "即将开始初始化标定，请保持设备静止！！！" << endl;
    int seconds = 3;
    cout << "倒计时: \n";
    for (int i = seconds; i>=0;--i)
    {
        cout << i << "s";
        cout.flush();
        this_thread::sleep_for(std::chrono::seconds(1));
        cout << "\b\b\b\b\b\b\b\b\b\b\b";
    }
    cout << "开始初始化imu----->"<< endl;
    // 待发送的报文
    uint8_t imu_send_data[] = {0x77, 0x04, 0x00, 0x59, 0x5D};
    int imu_send_size = sizeof(imu_send_data) / sizeof(uint8_t);

    // 接收回传报文
    std::string imu_response;

    double ax[100];
    double ay[100];
    double az[100];

    double r[100];
    double p[100];
    double y[100];

    int i;
    while(i<100)
    {
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

        ax[i] = read_acc(imu_response, 26) * 9.79134;
        ay[i] = read_acc(imu_response, 32) * 9.79134;
        az[i] = read_acc(imu_response, 38) * 9.79134;

        r[i] = -read_gyo(imu_response, 14) * M_PI / 180;
        p[i] = read_gyo(imu_response, 8) * M_PI / 180;
        y[i] = 0;
        // y[i] = -read_gyo(imu_response, 20) * M_PI / 180;

        i++;
        // 清理串口
        imu_ser.flushInput();
        imu_ser.flushOutput();
    }

    double acc_x,acc_y,acc_z;
    acc_x = average(ax);
    acc_y = average(ay);
    acc_z = average(az);

    double G_sum;
    G_sum = sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
    cout << "此处静止情况下的和加速度： " << G_sum << endl;

    double r_hat,p_hat,y_hat;
    r_hat = average(r);
    p_hat = average(p);
    y_hat = average(y);
    cout << "此时静止情况下的欧拉角：" << "\n" << "r: " << r_hat << "\n" << "p: " << p_hat << "\n" << "y: " << y_hat << endl;
    sleep(2);

    cv::Mat cbn = C_bn(r_hat, p_hat, y_hat);
    cout << "变换矩阵：\n" << cbn << endl;

    sleep(2);
    cout << "标定完成，等待AGV开始运动!" << endl;
    sleep(2);
    // int agv_mod = 1;
    // 声明比力
    cv::Mat acc = cv::Mat::ones(3,1,CV_32FC1);
    
    // 声明在地平导航坐标系下的运动加速度
    cv::Mat acc_n;
    // 声明上一时刻的运动加速度
    // cv::Mat acc_n_t = cv::Mat::zeros(3,1,CV_32FC1);


    auto time_t_0 = chrono::steady_clock::now();
    auto time_start = chrono::steady_clock::now();
    //
    float displacement_x = 0.000;
    float displacement_y = 0.000;
    
    float velocity_x = 0.000;
    float velocity_y = 0.000;

    double roll,pitch,yaw;
    float ax_n;
    float ay_n;

    ros::Rate loop_rate(1000);
    while(true)
    {
        ins_odom.header.stamp = ros::Time::now();

        ins_odom.header.frame_id = "ins_odom";

        
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

        roll = -read_gyo(imu_response, 14) * M_PI / 180;
        pitch = read_gyo(imu_response, 8) * M_PI / 180;
        yaw = 0;
        
        cv::Mat cbn_t = C_bn(roll, pitch, yaw);
        
        // cout << "此时的欧拉角为：\n" << " roll: \n  " <<  roll << "\n pitch:\n  " << pitch << "\n yaw:\n  " << yaw  << endl;

        

        // 将比力变换之导航坐标系下
        acc_n = cbn_t * acc;

        float ax_n_t = ax_n;
        float ay_n_t = ay_n;
        
        float ax_n = - acc_n.at<float>(0,0);
        float ay_n = - acc_n.at<float>(1,0);

        double dax_n = ax_n - ax_n_t;
        double day_n = ay_n - ay_n_t;



        // cout << "加速度为:\n" << " x轴:\n  " << ax_n << "\n y轴:\n  " << ay_n << endl;

        // 计算与上一时刻的时间差
        auto time_now = chrono::steady_clock::now();
        auto duration = chrono::duration_cast<std::chrono::milliseconds>(time_now - time_t_0);
        double time_dt = duration.count();
        time_t_0 = time_now;


        // -------------------------------------------------------------
        // 测试
        auto duration_start = chrono::duration_cast<std::chrono::milliseconds>(time_now - time_start);
        double time_2_start = duration_start.count() / 1000;
        float dt = time_dt/1000;
        // cout << "与上一时刻的时间差:\n" << " dt:\n  " << dt << "s" << endl;
        
        // -------------------------------------------------------------

        // 此处需要先通过ads获取agv的运动状态
        //-------------------------------------------------------------
        //----------------------------向前走----------------------------
        //-------------------------------------------------------------

        // cout << "上一时刻的总位移量:\n" << " x:\n  " << displacement_x << "\n y:\n  " << displacement_y << endl;
        float dx = 0.5 * ax_n * dt * dt;
        float dy = 0.5 * ay_n * dt * dt;
        // cout << "dt时间内的位移量:\n" << " x:\n  " << dx << "\n y:\n  " << dy << endl;
   
        // cout << "此时的总位移量:\n" << " x:\n  " << displacement_x << "\n y:\n  " << displacement_y << endl;
        
        float dv_x = ax_n * dt;
        float dv_y = ay_n * dt;

        velocity_x = velocity_x + dv_x;
        velocity_y = velocity_y + dv_y;

        if(abs(dax_n) <= 1e-5)
        {
            velocity_x = 0.0;
            dx = 0.0;
        }
        if(abs(day_n) <= 1e-5)
        {
            velocity_y = 0.0;
            dy = 0.0;
        }

        displacement_x = displacement_x + velocity_x*dt + dx;
        displacement_y = displacement_y + velocity_y*dt + dy; 
        
        
        ins_odom.twist.twist.linear.x = velocity_x;
        ins_odom.twist.twist.linear.y = velocity_y;
        ins_odom.pose.pose.position.x = displacement_x;
        ins_odom.pose.pose.position.y = displacement_y;


        imu_ser.flushInput();
        imu_ser.flushOutput();

        ins_pub.publish(ins_odom);

        //cout << "---" << endl;
        // usleep(200000);
        
        // sleep(1);
    }
    ros::spinOnce();
    loop_rate.sleep();




    

    // cout << "加速度矩阵：\n" << acc << endl;

    /* cv::Mat acc_cali;
    acc_cali = cbn * acc;
    cout << "此时静止情况下地平坐标系下的加速度：\n" << acc_cali << endl; */


    




    return 0;
}
