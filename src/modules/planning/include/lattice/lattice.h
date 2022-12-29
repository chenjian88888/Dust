/**
 * @file lattice.h 主要是订阅topic /referenceLine_smoothed，为参考线计算heading和kappa生成reference_point，之后进行速度规划和路径规划
 * @author feifei (gaolingfei@buaa.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-12-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __lattice__
#define __lattice__
#pragma once

#include<iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "reference_point.h"
#include "path_matcher.h"
#include "trajectoryPoint.h"
#include "gps.h"

namespace dust{
namespace lattice_ns{

class lattice
{
public:
    lattice();
    ~lattice() = default;

    /**
     * @brief 订阅平滑后的全局路径
     * 
     * @param path_point 
     */
    void referenceLineCallback(const nav_msgs::Path &path_point);
    /**
     * @brief gps的回调函数
     * 
     * @param pGps 
     */
    void gpsCallback(const msg_gen::gps &pGps)
    /**
     * @brief lattice模块规划入口
     * 
     */
    void plan();
    /**
     * @brief 计算规划起点
     * 
     */
    void plan_start_point();

private:
    // handle
    ros::NodeHandle n_;
    // publisher
  	// ros::Publisher ;
  	// subscriber
  	ros::Subscriber referenceLine_subscriber_, gps_sub_;

    // param
    std::vector<double> path_point_flag_; // 添加flag，确保参考线平滑一次
    double d0;                   //初始的横向偏移值 [m]
    double dd0;                  //初始的横向速度 [m/s]
    double ddd0;                 //初始的横向加速度 [m/s^2]
    double init_lon_state;       //初始的纵向值[m]
    double ds0;                  //初始的纵向速度[m/s]
    double dds0;                 //初始的纵向加速度[m/ss]
    double init_relative_time;   //规划起始点的时间
    double x_init;
    double y_init;
    double z_init;
    double v_init;
    double a_init;
    double theta_init;
    // double theta_end;
    double kappa_init;
    double dkappa_init;

    std::pair<std::vector<double>, std::vector<double>> reference_path; //参考路径点位置（x,y）
    std::vector<double> accumulated_s;                                  // 纵向距离

    // class
    DiscretizedTrajectory best_path_; //最佳路径
    std::vector<ReferencePoint> reference_points;                       // 参考路径点参数
    msg_gen::gps gps_;

    // flag
    bool is_first_run = true;
    std::vector<double> gps_flag_;
};


} // namespace lattice_ns
} // namespace dust

#endif // !__lattice__

