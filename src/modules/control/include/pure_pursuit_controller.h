#ifndef __PURE_PURSUIT__
#define __PURE_PURSUIT__

#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>

#include "gps.h"



struct RefPoint
{
	double x;
	double y;
	double kappa;
	double theta;
};

class purePursuit
{
public:
	purePursuit();
	~purePursuit() = default;

	void routingCallback(const geometry_msgs::PoseArray &routing);
	void gpsCallback(const msg_gen::gps &pGps);
	void run();

	double calculateCmd(const std::vector<RefPoint>& targetPath, const msg_gen::gps &gps);
};

#endif // !__PURE_PURSUIT__

