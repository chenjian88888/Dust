/**
 * @file routing_core.h
 * @author feifei (SY2113102@buaa.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-10-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ROUTING_CORE
#define ROUTING_CORE

#include <ros/ros.h>


namespace apollo {
namespace routing_core {

/**
 * @brief 全局规划的类
 * 
 */
class GlobalRouting
{

private:
    /* data */
public:
    GlobalRouting();
    ~GlobalRouting();
    /**
     * @brief 规划函数，传入起点和终点，输出最优路径
     * 
     * @param start_pose 
     * @param goal_pose 
     */
    void soultion(geometry_msgs::Pose &start_pose, geometry_msgs::Pose &goal_pose);
};
} // routing_core
} // apollo



#endif