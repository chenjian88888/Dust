#include <ros/ros.h>
#include "pure_pursuit_controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "control");
    purePursuit pp;
    pp.run();
}