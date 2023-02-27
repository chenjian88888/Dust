#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include "metaController.h"
#include "lqr_control.h"
#include "pure_pursuit_controller.h"

#include<iostream>
#include <deque>
#include <boost/thread.hpp>

using namespace dust::control;

int which_controllers;
std::shared_ptr<controller> control_base;
std::vector<RefPoint> targetPath_;
msg_gen::gps gps_;

// void routingCallback(const msg_gen::trajectory &routing){
// 	// 确保一开始只订阅一次
// 	std::cout << "routing.size: " << routing.pointsize << " targetPath_.size: " << targetPath_.size() << std::endl;
//   targetPath_.clear();
//   targetPath_.resize(routing.pointsize);
// 	for (int i = 0; i < routing.pointsize; ++i)
// 	{
// 		targetPath_[i] = {routing.trajectorypoint[i].x, routing.trajectorypoint[i].y, routing.trajectorypoint[i].kappa, routing.trajectorypoint[i].theta, routing.trajectorypoint[i].v};
// 	}
// }

void routingCallback(const geometry_msgs::PoseArray &routing){
	// 确保一开始只订阅一次
  if (targetPath_.size() < 1){
    ROS_INFO("1111");
    std::vector<std::pair<double, double>> xy_set;
    targetPath_.resize(routing.poses.size());
    xy_set.resize(routing.poses.size());
    for (int i = 0; i < routing.poses.size(); ++i)
    {
      xy_set[i] = {routing.poses[i].position.x, routing.poses[i].position.y};
    }
    // 差分
    std::deque<std::pair<double, double>> dxy;
    for (int i = 0; i < xy_set.size() - 1; ++i) {
      double dx = xy_set[i + 1].first - xy_set[i].first;
      double dy = xy_set[i + 1].second - xy_set[i].second;
      dxy.emplace_back(dx, dy);
    }
    std::deque<std::pair<double, double>> dxy_pre = dxy;
    std::deque<std::pair<double, double>> dxy_after = dxy;
    dxy_pre.emplace_front(dxy.front());// 加上第一个数
    dxy_after.emplace_back(dxy.back());// 加上最后一个数
    
    std::deque<std::pair<double, double>> dxy_final;
    for (int i = 0; i < xy_set.size(); ++i) {
      double dx = (dxy_pre[i].first + dxy_after[i].first) / 2;
      double dy = (dxy_pre[i].second + dxy_after[i].second) / 2;
      dxy_final.emplace_back(dx, dy);
    }
    // 计算heading
    std::deque<double> frenet_theta;
    std::vector<double> ds_final;
    for (int i = 0; i < xy_set.size(); ++i) {
      double theta = atan2(dxy_final[i].second, dxy_final[i].first);
      frenet_theta.push_back(theta);

      // 计算每一段的弧长
      double ds = sqrt(pow(dxy_final[i].second, 2) + pow(dxy_final[i].first, 2));
      ds_final.push_back(ds);
    }
    std::deque<double> dtheta;
    for (int i = 0; i < xy_set.size() - 1; ++i) {
      // 计算theta_diff
      double theta_diff = frenet_theta[i + 1] - frenet_theta[i];
      dtheta.push_back(theta_diff);
    }
    std::deque<double> dtheta_pre = dtheta;
    std::deque<double> dtheta_after = dtheta;
    dtheta_pre.push_front(dtheta.front());
    dtheta_after.push_back(dtheta.back());
    ROS_INFO("222222");
    for (int i = 0; i < xy_set.size(); ++i) {
      double theta_final = (dtheta_pre[i] + dtheta_after[i]) / 2;
      targetPath_[i] = {routing.poses[i].position.x, routing.poses[i].position.y, sin(theta_final) / ds_final[i], frenet_theta[i], 5};
      //std::cout << "theta: " << frenet_theta[i] << std::endl;
      // std::cout << sin(theta_final) / ds_final[i] << "\t";
    }
    // std::cout << std::endl;
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
      control_base = std::make_shared<lqrControl>(0.1,0.06,0.0);
      break;
    case 1:
      std::cout << "pure_pursuit init!!!" << std::endl;
      control_base = std::make_shared<purePursuit>(0.1,0.06,0.0);
      break;
    default:
      std::cout << "default:pure_pursuit init!!!" << std::endl;
      control_base = std::make_shared<purePursuit>(0.1,0.06,0.0);
      break;
  }

  ros::NodeHandle n_;
  // ros sub
  // ros::Subscriber planning_sub = n_.subscribe("/trajectory_waypoints", 10, routingCallback);
  ros::Subscriber planning_sub = n_.subscribe("/routing", 10, routingCallback);

  ros::Subscriber gps_sub = n_.subscribe("/gps", 10, gpsCallback);
  // ros pub
  ros::Publisher control_pub = n_.advertise<geometry_msgs::Pose>("/control", 10);
  ros::Publisher point_pub = n_.advertise<geometry_msgs::PointStamped>("/vehicle_gps",10);                // 发布车辆定位
  

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    // 发布车辆定位
    geometry_msgs::PointStamped this_point_stamped;
    this_point_stamped.header.stamp = ros::Time::now();
    this_point_stamped.header.frame_id = "world";
    this_point_stamped.point.x = gps_.posX;
    this_point_stamped.point.y = gps_.posY;
    this_point_stamped.point.z = gps_.posZ;
    point_pub.publish(this_point_stamped);

    // 先订阅到消息才可以发布 
    if (targetPath_.size() > 0) {
      double steer = control_base->calculateCmd(targetPath_, gps_);
      double speed = control_base->calculateThrottleBreak(targetPath_, gps_);
      
      geometry_msgs::Pose tempPose;
      tempPose.position.x = steer;
      tempPose.position.y = speed;
      control_pub.publish(tempPose);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}