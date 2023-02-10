#include "pure_pursuit_controller.h"


purePursuit::purePursuit(){
	ROS_INFO("pure pursuit start");
}

// void purePursuit::routingCallback(const geometry_msgs::PoseArray &routing){
// 	// 确保一开始只订阅一次
// 	targetPath_.resize(routing.poses.size());
// 	for (int i = 0; i < routing.poses.size(); ++i)
// 	{
// 		targetPath_[i] = {routing.poses[i].position.x, routing.poses[i].position.y, 0, 0};
// 	}
// }



void purePursuit::run(){
	
	
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
	for (int j = index; j < targetPath.size(); ++j) {
		forwardIndex = j;
		double distance = sqrtf((double)pow(targetPath[j].x - x, 2) +
			pow((double)targetPath[j].y - y, 2));
		if (distance >= progDist) {
			break;
		}
	}

	std::cout << "forwardIndex: " << forwardIndex << std::endl;
	double psi = gps.oriZ;
	double deltaAlfa = atan2(targetPath[forwardIndex].y - y,
		targetPath[forwardIndex].x - x) - psi;// alfa
	double ld = sqrt(pow(targetPath[forwardIndex].y - y, 2) +
		pow(targetPath[forwardIndex].x - x, 2)); // distance 
	double steer = -atan2(2. * (2.85) * sin(deltaAlfa), ld) * 36 / (7 * M_PI);
	double velocity = targetPath[index].kappa;
	std::cout << "velocity: " << velocity << "  steer: " << steer << std::endl;
	// if (steer > 135) {
	// 	steer = 135;
	// }
	// else if (steer < -135) {
	// 	steer = -135;
	// }
	return steer;
}