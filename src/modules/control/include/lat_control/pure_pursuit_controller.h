#ifndef __PURE_PURSUIT__
#define __PURE_PURSUIT__

#pragma once
#include "controller.h"

namespace dust {
namespace control {

class purePursuit : public controller
{
public:
	purePursuit(const double kp, const double ki, const double kd);
	~purePursuit() = default;

	void routingCallback(const msg_gen::trajectory &routing);
	// void routingCallback(const geometry_msgs::PoseArray &routing);
	void gpsCallback(const msg_gen::gps &pGps);
	void run();

	double calculateCmd(const std::vector<RefPoint>& targetPath, const msg_gen::gps &gps);
};

} // control
} // dust

#endif // !__PURE_PURSUIT__

