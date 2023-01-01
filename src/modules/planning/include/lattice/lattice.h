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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include "reference_point.h"
#include "path_matcher.h"
#include "trajectoryPoint.h"
#include "gps.h"
#include "PlanningTarget.h"
#include "Obstacle.h"
#include "cartesian_frenet_conversion.h"
#include "trajectory1d_generator.h"
#include "trajectory_evaluator.h"
#include "trajectory_combiner.h"
#include "constraint_checker.h"
#include "collision_checker.h"

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
    void gpsCallback(const msg_gen::gps &pGps);
    /**
     * @brief lattice模块规划入口
     *
     */
    void plan();
    /**
     * @brief 计算规划起点:第一次运行，规划起点就是定位点，轨迹的absolute_time=current_time；之后每次运行先判断控制是否跟上
     * 控制跟上：规划起点是轨迹绝对时间上往后0.1s的轨迹点，absolute_time是在上一帧轨迹中找到距current_time+0.1最接近的轨迹点的索引
     * 控制没跟上：规划起点根据车辆动力学合理外推，absolute_time=current_time+0.1;
     *
     */
    void plan_start_point(double &tt);
    /**
     * @brief lattice主函数入口
     * 
     * @param planning_init_point 
     * @param planning_target 
     * @param obstacles 
     * @param accumulated_s 
     * @param reference_points 
     * @param lateral_optimization 
     * @param init_relative_time 
     * @param lon_decision_horizon 
     * @return DiscretizedTrajectory 
     */
    DiscretizedTrajectory LatticePlan(
      const TrajectoryPoint &planning_init_point,
      const PlanningTarget &planning_target,
      const std::vector<const Obstacle *> &obstacles,
      const std::vector<double> &accumulated_s,
      const std::vector<ReferencePoint> &reference_points, const bool &lateral_optimization,
      const double &init_relative_time, const double lon_decision_horizon, const double &absolute_time);
    
    /**
     * @brief 规划起点frenet和cartesian坐标转换
     * 
     * @param matched_point 
     * @param cartesian_state 
     * @param ptr_s 
     * @param ptr_d 
     */
    void ComputeInitFrenetState(const ReferencePoint &matched_point, const TrajectoryPoint &cartesian_state,
                                std::array<double, 3> *ptr_s, std::array<double, 3> *ptr_d);

  public:
    std::vector<ReferencePoint> reference_points;                       // 参考路径点参数
    msg_gen::gps gps_;
    std::vector<double> gps_flag_;

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
    double s0;                   //初始的纵向值[m]
    double ds0;                  //初始的纵向速度[m/s]
    double dds0;                 //初始的纵向加速度[m/ss]
    double init_relative_time;   //规划起始点的时间
    double absolute_time;        //轨迹上的绝对时间
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
    double lon_decision_horizon = 0;// 前视距离
    InitialConditions lattice_ic_;

    // class
    DiscretizedTrajectory best_path_; //最佳路径
    TrajectoryCombiner trajectorycombiner_;
    ConstraintChecker constraintchecker_;

    // flag
    bool is_first_run = true;
    bool FLAGS_lateral_optimization; //选择二次规划
};


} // namespace lattice_ns
} // namespace dust

#endif // !__lattice__

