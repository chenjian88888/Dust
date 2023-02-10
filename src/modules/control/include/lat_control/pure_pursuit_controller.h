#ifndef __PURE_PURSUIT__
#define __PURE_PURSUIT__

#pragma once






class purePursuit
{
public:
	purePursuit();
	~purePursuit() = default;

	void routingCallback(const msg_gen::trajectory &routing);
	// void routingCallback(const geometry_msgs::PoseArray &routing);
	void gpsCallback(const msg_gen::gps &pGps);
	void run();

	std::array<double, 2> calculateCmd(const std::vector<RefPoint>& targetPath, const msg_gen::gps &gps);
};

#endif // !__PURE_PURSUIT__

