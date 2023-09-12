#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

#include <stdio.h>
#include <math.h>

float x_0,y_0;
int i = 0;

void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    

    float x =  msg->pose.position.x;
    float y =  msg->pose.position.y;

    if(i == 0)
    {
        x_0 = x;
        y_0 = y;   
    }

    float displacement = sqrt((x-x_0) * (x-x_0) + (y-y_0) * (y-y_0));

    std::cout << "位移:\n " << displacement << std::endl;

    i++;
    std::cout << "----------" << std::endl;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "displacement_calc");

    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/tracked_pose", 10, pose_cb);

    ros::spin();

    return 0;
}