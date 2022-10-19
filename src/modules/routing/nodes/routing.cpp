/**
 * @file routing.cpp
 * @author feifei (SY2113102@buaa.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-10-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_routing");
    GlobalRouting globalRouting;
    globalRouting.mainloop();
    return 0;
}