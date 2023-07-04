#include <ros/ros.h>
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

#include <fstream>

#include <nav_msgs/Odometry.h>

using namespace std;

nav_msgs::Odometry motor_data;
// 设置串口数据
serial::Serial motor_ser("/dev/ttyUSB1", 9600, serial::Timeout::simpleTimeout(100));

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_serial_node");

    ros::NodeHandle nh;

    ros::Publisher motor_pub = nh.advertise<nav_msgs::Odometry>("/odom0",100);

    // 待发送的报文
    uint8_t motor_send_data[] = {0x01, 0x03, 0x00, 0x57, 0x00, 0x0C, 0xF4, 0x1F};
    int motor_send_size = sizeof(motor_send_data) / sizeof(uint8_t);

    // 接收回传的报文
    std::string motor_response;

    ros::Rate loop_rate(1000);

    double motor_pos_b = 0.0;
    auto time_before = chrono::steady_clock::now();

    while(ros::ok())
    {
        // 设置电机消息的时间戳
        motor_data.header.stamp = ros::Time::now();
        // 设置电机的坐标系
        motor_data.header.frame_id = "motor_odom";
        
        
        // 将待发送的报文发送至串口 
        motor_ser.write(motor_send_data, motor_send_size);

        // 接收回传的报文
        vector<uint8_t> motor_recv_data;
        
        // 读取串口返回的数据，大小为29个bytes
        motor_ser.read(motor_recv_data,29);

        // 如果接收到串口返回的数据大小不足29bytes，则跳过此次循环
        if (motor_recv_data.size() != 29)
        {
            // cout << "Error: Failed to read 5 bytes from serial port." << endl;
            continue;
        }
        
        // 定义电机位置
        int motor_pos;
        // 将返回数据通过地址转换为整形数后传入变量
        vector<uint8_t> motor_pos_data = {motor_recv_data[6],motor_recv_data[5],motor_recv_data[4],motor_recv_data[3]};
        memcpy(&motor_pos, &motor_pos_data[0], sizeof(motor_pos));

        
        // 如果超出量程默认为-1（通常是这样的）
        if (motor_pos > 1300000)
        {
            motor_pos = -1;
        }


        // 获取当前时间，并dt用于计算速度
        auto time_now = chrono::steady_clock::now();
        auto duration = chrono::duration_cast<std::chrono::milliseconds>(time_now - time_before);
        double dt = duration.count();
        // 重置前一刻时间，此刻成上一刻
        time_before = time_now;
        
        // cout << motor_pos << " " << motor_pos_b << endl;
        // 通过反馈位置计算出速度
        double motor_vel = (motor_pos - motor_pos_b)/dt;

        motor_pos_b = motor_pos;

        // 将数据传入odomtry消息
        motor_data.twist.twist.linear.x = motor_vel/1000;
        motor_data.pose.pose.position.x = (double)motor_pos/1000000;

        // 清理串口
        motor_ser.flushInput();
        motor_ser.flushOutput();

        motor_pub.publish(motor_data);
    }

    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}
