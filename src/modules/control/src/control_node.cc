#include <ros/ros.h>
#include "controller.h"

using namespace dust::contorl;

int which_controllers;
std::shared_ptr<controller> control_base;
std::vector<RefPoint> targetPath_;
msg_gen::gps gps_;

void routingCallback(const msg_gen::trajectory &routing){
	// 确保一开始只订阅一次
	std::cout << "routing.size: " << routing.pointsize << " targetPath_.size: " << targetPath_.size() << std::endl;
	targetPath_.resize(routing.pointsize);
	for (int i = 0; i < routing.pointsize; ++i)
	{
		targetPath_[i] = {routing.trajectorypoint[i].x, routing.trajectorypoint[i].y, routing.trajectorypoint[i].kappa, routing.trajectorypoint[i].theta};
	}
}

void gpsCallback(const msg_gen::gps &pGps){
	gps_ = pGps;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control");

    // ros parameter settings
    ros::param::get("which_controllers", which_controllers);
    ROS_INFO("which_controllers = %d",which_controllers);
    switch (which_controllers)
    {
      case 0:
        std::cout << "lqr init!!!" << std::endl;
        control_base = std::make_shared<lqrControl>();
        break;
      case 1:

        break;
      default:
        std::cout << "default:lqr init!!!" << std::endl;
        control_base = std::make_shared<lqrControl>();
        break;
    }

    ros::NodeHandle n_;
    // ros sub
	ros::Subscriber planning_sub = n_.subscribe("/trajectory_waypoints", 10, routingCallback);
	// ros::Subscriber planning_sub = n_.subscribe("/routing", 10, &purePursuit::routingCallback, this);

	ros::Subscriber gps_sub = n_.subscribe("/gps", 10, gpsCallback);

    ros::Publisher control_pub = n_.advertise<geometry_msgs::Pose>("/dynamic_waypoints", 10);

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		// 先订阅到消息才可以发布
		
		if (targetPath_.size() > 0) {
            double steer = control_base.calculateCmd(targetPath_, gps_);
            std::array<double,2> sv = calculateCmd(targetPath_, gps_);
			geometry_msgs::Pose tempPose;
			tempPose.position.x = sv[0];
			tempPose.position.y = sv[1];
			control_pub.publish(tempPose);
		}
		ros::spinOnce();
        loop_rate.sleep();
	}

    purePursuit pp;
    pp.run();
}