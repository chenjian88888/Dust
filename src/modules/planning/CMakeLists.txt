cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic,melodic
##Compile as C++14, supported in ROS nortic
add_compile_options(-std=c++14)

set(SRC_FILE
    src/reference_line/reference_line.cc
    src/reference_line/cos_theta_ipopt_interface.cc
    src/reference_line/fem_pos_deviation_ipopt_interface.cc
    src/reference_line/fem_pos_deviation_osqp_interface.cc
    src/reference_line/fem_pos_deviation_sqp_osqp_interface.cc
    src/reference_line/reference_point.cc
    src/planning.cc

    src/common/math_utils.cc
    src/common/vec2d.cc
    src/common/PlanningTarget.cc

    src/lattice/lattice.cc

    src/path/path_matcher.cc
    src/path/path_points.cc
    src/path/trajectoryPoint.cc
    

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
    include/common
    include/lattice
    include/msg_gen
    include/path
    include/planning.h
    include/reference_line
    include ${catkin_INCLUDE_DIRS}
)

add_executable(planning_node ${SRC_FILE})
target_link_libraries(planning_node ${catkin_LIBRARIES} -lxerces-c osqp adolc ipopt)