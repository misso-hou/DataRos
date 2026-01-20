#include "function/ros_topic_parser.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "dbw_reports_listener");
    
    // 创建监听器对象
    MsgParser msg_parser;

    ROS_INFO("ROS listener started, waiting for messages on /vehicle/dbw_reports...");
    
    // 进入ROS事件循环
    ros::spin();
    
    return 0;
}