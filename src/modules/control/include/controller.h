/**
 * @file controller.h
 * @author feifei (gaolingfei@buaa.edu.cn)
 * @brief control base class, lateral && longitudinal
 * @version 0.1
 * @date 2023-02-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _CONTROL_
#define _CONTROL_

#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>

#include "gps.h"
#include "trajectory.h"
#include "TrajectoryPoint.h"

#include <Eigen\Dense>


namespace dust{
namespace contorl{

struct RefPoint
{
	double x;
	double y;
	double kappa;
	double theta;
};

/**
 * @brief control class, lateral && longitudinal
 * 
 */
class controller
{
public:
	controller(const double kp, const double ki, const double kd);
	virtual ~controller()=default;

	// virtual function, base, lqrControl and pure_suit is inherited
	virtual double calculateCmd(const std::vector<RefPoint> &targetPath, const msg_gen::gps &gps) = 0;

public:
	// calc forwardindex
	int calc_forwardIndex(const std::vector<RefPoint>& targetPath, const msg_gen::gps &gps);

	double calculateThrottleBreak(const std::vector<RefPoint>& targetPath, const msg_gen::gps &gps, size_t forwardIndex);

	double PID_Control(double value_target, double value_now);

	double calculateKappa(const std::vector<RefPoint>& targetPath, int idx);

	void reset();

protected:
	double kp_ = 0.0;
	double ki_ = 0.0;
	double kd_ = 0.0;
	double error_sub_ = 0.0;
	double previous_error_ = 0.0;
	double integral_ = 0.0;
	double differential_ = 0.0;
	bool first_init_ = false;
};

} // control
} // dust

#endif // !_CONTROL_