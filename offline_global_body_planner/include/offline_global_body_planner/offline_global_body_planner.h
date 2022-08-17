#ifndef OFFLINE_GLOBAL_BODY_PLANNER_H
#define OFFLINE_GLOBAL_BODY_PLANNER_H

#include <nav_msgs/Path.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// #include "global_body_planner/gbpl.h" // Don't think I need this
// #include "global_body_planner/global_body_plan.h" // Probably also do not need this


void printHelloWorld();

#endif // OFFLINE_GLOBAL_BODY_PLANNER_H