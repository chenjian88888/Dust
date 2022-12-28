/**
 * @file reference_line.h
 * @author feifei (gaolingfei@buaa.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-12-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __reference_line__
#define __reference_line__
#pragma once

#include<iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <utility>
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>

#include "cos_theta_ipopt_interface.h"
#include "fem_pos_deviation_osqp_interface.h"
#include "fem_pos_deviation_sqp_osqp_interface.h"
#include "fem_pos_deviation_ipopt_interface.h"
#include "lattice/lattice.h"

namespace dust{
namespace reference_line{
	
// std::vector<RefPoint> targetPath_;
struct Point3d_s
{
  double x;
  double y;
  double z;
};

class referenceLine
{
public:
	referenceLine();
	~referenceLine() = default;
	
	/**
	 * @brief 全局路径的回调函数
	 * 
	 * @param routing 
	 */
	void routingCallback(const geometry_msgs::PoseArray &routing);
	/**
	 * @brief planning的入口函数
	 * 
	 */
	void run();

	/**
	 * @brief 对参考线进行操作
	 * 
	 * @param hdmap_way_points 
	 */
	void referenceLine_split(Eigen::MatrixXd &hdmap_way_points);
	/**
	 * @brief 将参考线以话题 发布
	 * 
	 * @param path_point_after_interpolation_ 
	 */
	void Smooth(Eigen::MatrixXd &path_point_after_interpolation_);
	void NormalizePoints(std::vector<std::pair<double, double>> *xy_points);
	void DeNormalizePoints(std::vector<std::pair<double, double>> *xy_points);

	/**
	 * @brief 将输入的全局路径插值后输出到output
	 * 
	 * @param input 
	 * @param output 
	 * @param interval_dis 
	 * @param distance 
	 */
	void average_interpolation(Eigen::MatrixXd& input, Eigen::MatrixXd& output, double interval_dis, double distance);

private:
	bool CosThetaSmooth(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
             std::vector<double>* opt_x, std::vector<double>* opt_y);
	/**
	 * @brief femSmooth方法
	 * 
	 * @param raw_point2d 
	 * @param bounds 
	 * @param opt_x 
	 * @param opt_y 
	 * @return true 
	 * @return false 
	 */
	bool FemPosSmooth(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
             std::vector<double>* opt_x, std::vector<double>* opt_y);

	bool QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
                  std::vector<double>* opt_x, std::vector<double>* opt_y);

	bool NlpWithIpopt(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
						std::vector<double>* opt_x, std::vector<double>* opt_y);

	bool SqpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
					std::vector<double>* opt_x, std::vector<double>* opt_y);

private:
	// handle
	ros::NodeHandle n_;
	// publisher
  	ros::Publisher referenceLine_pub_;
  	// subscriber
  	ros::Subscriber routing_sub_;

	// param
	Eigen::MatrixXd routing_waypoints_;// 中心点坐标
	Eigen::MatrixXd path_point_after_interpolation_;// 插值后的轨迹点
	double zero_x_ = 0.0;
	double zero_y_ = 0.0;
	nav_msgs::Path referenceline_;
	bool which_smoothers;  //选择参考线平滑方式
	/*
	如果考虑参考线的曲率约束，其优化问题是非线性的，
	可以使用ipopt非线性求解器求解（内点法），也可以使用osqp二次规划求解器来用SQP方法求解；
	如果不考虑曲率约束，则直接用osqp求解二次规划问题。
	*/
	bool apply_curvature_constraint;  //是否使用曲率约束
	bool use_sqp = false;                     //是否使用SQP方法求解
	std::vector<double> routing_waypoint_flag_; // 添加flag，确保参考线平滑一次
};

} // namespace reference_line
} // namespace dust

#endif // !__reference_line__

