cmake_minimum_required(VERSION 3.0.2)
project(control)

## Compile as C++11, supported in ROS Kinetic,melodic
##Compile as C++14, supported in ROS nortic
add_compile_options(-std=c++14)

set(SRC_FILE
    src/pure_pursuit_controller.cc
    src/control_node.cc

)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(FILES
    
)

generate_messages(DEPENDENCIES
    std_msgs
)

find_package(Eigen3 QUIET)
find_package(OsqpEigen QUIET)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
     geometry_msgs
     roscpp 
     rospy 
     std_msgs 
     message_runtime
    
    DEPENDS
    Boost
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    include/pure_pursuit_controller.h
    include/msg_gen
    include ${catkin_INCLUDE_DIRS}
)

add_executable(control_node ${SRC_FILE})
target_link_libraries(control_node ${catkin_LIBRARIES} -lxerces-c osqp adolc ipopt)