#include<ros/ros.h>
#include<std_msgs/String.h>
#include<iostream>

void callback(const std_msgs::String::ConstPtr &msg)
{
    std::cout << (msg->data) << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chatter_sub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("chatter", 1000, &callback);
    ros::spin();
    return 0;
}