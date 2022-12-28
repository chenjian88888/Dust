#include "pure_pursuit_controller.h"

std::vector<RefPoint> targetPath_;
msg_gen::gps gps_;
purePursuit::purePursuit(){

}

void purePursuit::routingCallback(const geometry_msgs::PoseArray &routing){
	// 确保一开始只订阅一次
	if (targetPath_.size() < 1){
		for (int i = 0; i < routing.poses.size(); ++i)
		{
			targetPath_.push_back({routing.poses[i].position.x, routing.poses[i].position.y, 0, 0});
		}
	}
}

void purePursuit::gpsCallback(const msg_gen::gps &pGps){
	gps_ = pGps;
}

void purePursuit::run(){
	ROS_INFO("pure pursuit start");
	ros::NodeHandle n_;
	ros::Subscriber routing_sub = n_.subscribe("/routing", 10, &purePursuit::routingCallback, this);
	ros::Subscriber gps_sub = n_.subscribe("/gps", 10, &purePursuit::gpsCallback, this);

    ros::Publisher control_pub = n_.advertise<geometry_msgs::Pose>("/dynamic_waypoints", 10);
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		// 先订阅到消息再可以发布
		
		if (targetPath_.size() > 0) {
			double steer = calculateCmd(targetPath_, gps_);
			geometry_msgs::Pose tempPose;
			tempPose.position.x = steer;
			control_pub.publish(tempPose);
		}
		ros::spinOnce();
        loop_rate.sleep();
	}
	
}

double purePursuit::calculateCmd(const std::vector<RefPoint>& targetPath, const msg_gen::gps &gps) {
	
	int index = 0;

	int forwardIndex = 0;
	double minProgDist = 3.0;
	double progTime = 0.6;
	double mainVehicleSpeed = sqrtf(gps.velX * gps.velX + gps.velY * gps.velY + gps.velZ * gps.velZ);
	double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;
	double x = gps.posX;
	double y = gps.posY;

	// find nearest index
	double min_dis = (std::numeric_limits<int>::max)();
	for (int i = 0; i < targetPath.size(); i++)
	{
		double dis = pow(targetPath[i].x - x, 2) + pow(targetPath[i].y - y, 2);
        if (dis < min_dis) {

            min_dis = dis;
            index = i;
        }
	}
	
	// find forwardIndex
	for (; index < targetPath.size(); ++index) {
		forwardIndex = index;
		double distance = sqrtf((double)pow(targetPath[index].x - x, 2) +
			pow((double)targetPath[index].y - y, 2));
		if (distance >= progDist) {
			break;
		}
	}

	// std::cout << "forwardIndex: " << forwardIndex << std::endl;
	double psi = gps.oriZ;
	double deltaAlfa = atan2(targetPath[forwardIndex].y - y,
		targetPath[forwardIndex].x - x) - psi;// alfa
	double ld = sqrt(pow(targetPath[forwardIndex].y - y, 2) +
		pow(targetPath[forwardIndex].x - x, 2)); // distance 
	double steer = -atan2(2. * (2.85) * sin(deltaAlfa), ld) * 36 / (7 * M_PI);
	std::cout << "steer: " << steer << std::endl;
	// if (steer > 135) {
	// 	steer = 135;
	// }
	// else if (steer < -135) {
	// 	steer = -135;
	// }
	return steer;
}