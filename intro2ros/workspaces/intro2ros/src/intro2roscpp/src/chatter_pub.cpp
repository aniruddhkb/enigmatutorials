#include<ros/ros.h>
#include<std_msgs/String.h>
#include<iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chatter_pub");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate rate(2);

    while(ros::ok())
    {
        std::string str;
        getline(std::cin,str);

        std::string nodeName = ros::this_node::getName();

        std_msgs::String msg;
        msg.data = nodeName + ": " + str;

        pub.publish(msg);
        rate.sleep();
    }
    return 0;   
}