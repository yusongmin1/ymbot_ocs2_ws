//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include <iostream>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf2_listener");
    ros::NodeHandle nh;

    // 创建一个tf2::Buffer对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
        auto cnm=buffer.allFramesAsString();
    std::cout<<cnm;
    ros::spinOnce(); // 必须调用一次spinOnce来启动监听

    return 0;
}