#include "../ads_include/AdsLib/AdsLib.h"
#include "../ads_include/AdsLib/AdsNotificationOOI.h"
#include "../ads_include/AdsLib/AdsVariable.h"

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"

#include <opencv/cv.h>
// #include <opencv4/opencv2/opencv.hpp>
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

int main(int argc, char** argv)
{
    // ------------------------------------------------------------
    // ----------------------初始化ads通讯的目标----------------------
    // ------------------------------------------------------------
    static const AmsNetId remoteNetId { 169, 254, 27, 166, 1, 1 };
    static const char remoteIpV4[] = "169.254.27.166";
    AdsDevice route {remoteIpV4, remoteNetId, AMSPORT_R0_PLC_RTS1};

    // 行进状态  
    // 0: 停止
    // 1: 前进
    // 前进状态
    int32_t movement_state_forward;
    // 后退状态
    int32_t movement_state_back;
    // 左移状态
    int32_t movement_state_left;
    // 右移状态
    int32_t movement_state_right;

    // 定义用于判断运动或静止的状态
    int movement_state;

    
}