#include <ros/ros.h>

#include "planning.h"

using namespace dust::reference_line;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    ROS_INFO("planning start");
    referenceLine rl;
    rl.run();    
}