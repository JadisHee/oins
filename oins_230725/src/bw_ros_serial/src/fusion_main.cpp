#include "../ads_include/AdsLib/AdsLib.h"
#include "../ads_include/AdsLib/AdsNotificationOOI.h"
#include "../ads_include/AdsLib/AdsVariable.h"

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"

// #include <opencv/cv.h>
#include <opencv4/opencv2/opencv.hpp>
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
 * @brief 用于通过姿态解算出由机体系转换至地平坐标系下的姿态矩阵
 * @param r 滚转角
 * @param p 俯仰角
 * @param y 偏航角
 * @return 姿态矩阵
 */
cv::Mat C_bn(double r, double p, double y)
{
    cv::Mat cbn = cv::Mat::ones(3, 3, CV_32FC1);
    cbn.at<float>(0, 0) = cos(p) * cos(y);
    cbn.at<float>(0, 1) = sin(r) * sin(p) * cos(y) - cos(r) * sin(y);
    cbn.at<float>(0, 2) = cos(r) * sin(p) * cos(y) + sin(r) * sin(y);

    cbn.at<float>(1, 0) = cos(p) * sin(y);
    cbn.at<float>(1, 1) = sin(r) * sin(p) * sin(y) + cos(r) * cos(y);
    cbn.at<float>(1, 2) = cos(r) * sin(p) * sin(y) - sin(r) * cos(y);

    cbn.at<float>(2, 0) = -sin(p);
    cbn.at<float>(2, 1) = sin(r) * cos(p);
    cbn.at<float>(2, 2) = cos(r) * cos(p);

    return cbn;
}

float x_0, y_0;
int i = 0;
float displacement_slam;
float x_slam, y_slam;

void pose_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{

    float x = msg->pose.position.x;
    float y = msg->pose.position.y;

    x_slam = msg->pose.position.x;
    y_slam = msg->pose.position.y;
        if (i == 0)
    {
        x_0 = x;
        y_0 = y;
    }

    displacement_slam = sqrt((x - x_0) * (x - x_0) + (y - y_0) * (y - y_0));

    // std::cout << "位移:\n " << displacement << std::endl;

    i++;
    // std::cout << "----------" << std::endl;
}

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "displacement_calc");

    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/tracked_pose", 10, pose_cb);

    ofstream outfile("/home/main/oins/oins_230725/src/bw_ros_serial/data/230907/record_3.csv");
    outfile << "行驶状态,"
            << "编码器速度,"
            << "惯性导航解算速度,"
            << "slam位移,"
            << "惯性导航解算位移,"
            << "编码器位移,"
            << "qr_code_num,"
            << "qr_code_x,"
            << "qr_code_y,"
            << "qr_code_r,"
            << "x_slam,"
            << "y_slam,"
            << "组合导航位移\n";

    // ------------------------------------------------------------
    // ----------------------初始化ads通讯的目标----------------------
    // ------------------------------------------------------------
    static const AmsNetId remoteNetId{169, 254, 27, 166, 1, 1};
    static const char remoteIpV4[] = "169.254.27.166";
    AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_RTS1};

    // 行进状态
    // 0: 停止
    // 1: 前进
    // 2: 后退
    int16_t movement_state;

    int movement_state_1;

    // 读取的编码器速度
    float encoder_speed;
    float calc_encoder_speed;


    float qr_code_num;
    float qr_code_x;
    float qr_code_y;
    float qr_code_r;
    // ------------------------------------------------------------
    // ----------------------初始化惯性导航的内容----------------------
    // ------------------------------------------------------------
    float threshold_ax_0 = 0.0075;
    // 0.0075

    float roll, pitch, yaw;

    // 待发送的报文
    uint8_t imu_send_data[] = {0x77, 0x04, 0x00, 0x59, 0x5D};
    int imu_send_size = sizeof(imu_send_data) / sizeof(uint8_t);

    // 接收回传报文
    std::string imu_response;

    // 声明变换矩阵
    cv::Mat cbn;

    // 声明比力
    cv::Mat acc_imu = cv::Mat::ones(3, 1, CV_32FC1);

    // 声明在地平导航坐标系下的运动加速度
    cv::Mat acc;

    // 声明起始时间和上一时刻的时间
    auto time_start = chrono::steady_clock::now();
    auto time_t_0 = chrono::steady_clock::now();

    float ax, ay, az;
    float ax_t, ay_t;
    float dax, day;

    float velocity_x = 0.0, velocity_y = 0.0;
    // float velocity_x_t, velocity_y_t;

    float displacement_x = 0.0, displacement_y = 0.0;
    // float displacement_x_t, displacement_y_t;

    float encoder_displacement_x = 0.0, encoder_displacement_y = 0.0;

    // ------------------------------------------------------------
    // ----------------------kalman--------------------------------
    // ------------------------------------------------------------

    float X = 0;
    float A = 1;
    float B;
    float H = 1;
    float P = 0;

    float Q = 0.01;
    float R = 1;

    movement_state = 1;
    while (true)
    {
        //
        AdsVariable<int16_t> state{route, 0X4020, 108};
        state = movement_state;
//         movement_state = state;

        AdsVariable<float> speed{route, 0X4020, 110};
        encoder_speed = speed / 1000;

        AdsVariable<uint32_t> qr_num{route, 0X4020, 312};
        qr_code_num = qr_num;

        AdsVariable<float> qr_x{route, 0X4020, 300};
        qr_code_x = qr_x/10000;

        AdsVariable<float> qr_y{route, 0X4020, 304};
        qr_code_y = qr_y/10000;

        AdsVariable<float> qr_r{route, 0X4020, 308};
        qr_code_r = qr_r;

        if (encoder_speed <= 0.0015)
        {
            movement_state_1 = 0;
        }
        else
        {
            movement_state_1 = 1;
        }

        if (movement_state_1 == 1)
        {
            calc_encoder_speed = encoder_speed + 0.0158;
        }
        else
        {
            calc_encoder_speed = encoder_speed;
        }

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
        imu_response.resize(48 * 2);
        static constexpr char hex_digits[] = "0123456789ABCDEF";
        for (std::size_t i = 0; i < 48; ++i)
        {
            imu_response[i * 2] = hex_digits[imu_recv_data[i] >> 4];
            imu_response[i * 2 + 1] = hex_digits[imu_recv_data[i] & 0xF];
        }

        roll = -read_gyo(imu_response, 14) * M_PI / 180;
        pitch = read_gyo(imu_response, 8) * M_PI / 180;
        yaw = -read_gyo(imu_response, 20) * M_PI / 180;

        acc_imu.at<float>(0, 0) = read_acc(imu_response, 26) * 9.79134;
        acc_imu.at<float>(1, 0) = read_acc(imu_response, 32) * 9.79134;
        acc_imu.at<float>(2, 0) = read_acc(imu_response, 38) * 9.79134;

        // 计算变换矩阵
        cbn = C_bn(roll, pitch, yaw);

        // 将比力变换至地平坐标系
        acc = cbn * acc_imu;

        // imu所测出的比力
        float ax_b = acc_imu.at<float>(0, 0);
        float ay_b = acc_imu.at<float>(1, 0);
        float az_b = acc_imu.at<float>(2, 0);

        // 计算两帧之间的时间差
        auto timeow = chrono::steady_clock::now();
        auto duration = chrono::duration_cast<std::chrono::milliseconds>(timeow - time_t_0);
        double time_dt = duration.count();
        time_t_0 = timeow;
        float dt = time_dt / 1000;

        ax_t = ax;
        ay_t = ay;

        // 地平坐标系下的加速度
        ax = -acc.at<float>(0, 0);
        ay = -acc.at<float>(1, 0);
        az = -acc.at<float>(2, 0);

        dax = abs(ax - ax_t);
        day = abs(ay - ay_t);

        if (movement_state_1 == 1)
        {
            ax = ax + threshold_ax_0;
        }

        float dx = 0.5 * ax * dt * dt;
        float dvx = ax * dt;
        /*
                if(dax <= 0.01)
                {
                    dvx = 0.0;
                    dx = 0.0;------------------------------------------------------------
                }
        */
        // velocity_x = velocity_x_t;

        velocity_x = velocity_x + dvx;

        encoder_displacement_x = encoder_displacement_x + dt * calc_encoder_speed;

        // velocity_y = velocity_y_t + dvy;
        if (dax <= 0.005 && abs(ax) <= 0.01 && movement_state_1 == 0)
        // if (dax <= 0.001 && abs(velocity_x) <= 0.05 && abs(ax) <= 0.01)
        {
            velocity_x = 0;
        }

        // ------------------------------------------------------------
        // --------------------------Kalman----------------------------
        // ------------------------------------------------------------
        B = dt;

        float X_predict = X + B * velocity_x;
        float P_predict = P + Q;

        float K = P_predict / (P_predict + R);
        X = X_predict + K * (encoder_displacement_x - X_predict);
        P = (1 - K) * P_predict;
        // ------------------------------------------------------------
        // ------------------------------------------------------------

        displacement_x = displacement_x + velocity_x * dt + dx;

        ros::spinOnce();

        cout << "此时的采样间隔为: \n"
             << dt << " s\n"
             << endl;
        cout << "车辆状态为: \n"
             << movement_state_1 << "\n"
             << endl;
        cout << "编码器读取速度为: \n"
             << encoder_speed << " m/s\n"
             << endl;
        // cout << "欧拉角为: \n" << " roll:\n  " << roll << "\n pitch:\n  " << pitch << "\n yaw:\n  " << yaw << endl;
        // cout << "机体坐标系下比力为:\n" << " x轴:\n  " << ax_b << "\n y轴:\n  " << ay_b << "\n z轴:\n  " << az_b << endl;
        // cout << "地平坐标系下的加速度为：\n" << " x轴:\n  " << ax << "\n y轴:\n  " << ay <<  "\n z轴:\n  " << az << endl;
        // cout << "加速度的帧间差为:\n" << " dax:\n  " << dax << endl;
        cout << "惯性导航解算速度：\n"
             << " x轴:\n  " << velocity_x << "\n"
             << endl;
        cout << "惯性导航位移：\n"
             << displacement_x << "\n"
             << endl;
        cout << "编码器位移：\n"
             << encoder_displacement_x << "\n"
             << endl;
        cout << "组合导航位移: \n"
             << X << "\n"
             << endl;
        // cout << "i: " << i << "\n" << endl;
        cout << "slam位移: \n"
             << displacement_slam << "\n" 
             << endl;

        cout << "slam_position: \n"
             << " x:\n  " << x_slam << "\n"
             << " y:\n  " << y_slam << "\n"
             << endl;

        outfile << movement_state_1 << ","
                << encoder_speed << ","
                << velocity_x << ","
                << displacement_slam << ","
                << displacement_x << ","
                << encoder_displacement_x << ","
                << qr_code_num << ","
                << qr_code_x << ","
                << qr_code_y << ","
                << qr_code_r << ","
                << x_slam << ","
                << y_slam << ","
                << X << "\n";

        if(displacement_slam >= 4.8)
        {
            movement_state = 0;
        }

    }

    outfile.close();
    return 0;
}