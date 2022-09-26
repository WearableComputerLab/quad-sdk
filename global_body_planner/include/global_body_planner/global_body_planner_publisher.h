#ifndef GLOBAL_BODY_PLANNER_PUBLISHER_H
#define GLOBAL_BODY_PLANNER_PUBLISHER_H

#include <nav_msgs/Path.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>  // May not need immediately (delete once integrated)

#include <grid_map_core/grid_map_core.hpp>  // Probably don't need grid map until I want to incorporate terrain information into a planner
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// #include "global_body_planner/gbpl.h" // Don't need prob
#include "global_body_planner/global_body_plan.h"

using namespace planning_utils;

class GlobalBodyPlannerPublisher {
 public:
  /**
   * @brief Construct a new Global Body Planner Publisher object
   *
   * @param nh Node handle
   * @return Constructed object of type GlobalBodyPlannerPublisher
   */
  GlobalBodyPlannerPublisher(ros::NodeHandle nh);

  /**
   * @brief Initiates a plan
   *
   * @return true
   * @return false
   */
  bool callPlanner();

  /**
   * @brief Custom spin
   *
   */
  void spin();

 private:
  /**
   * @brief Callback function to handle new terrain map data
   *
   * @param msg the message containing map data
   */
  void terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg);

  /**
   * @brief Callback function to handle new robot state data
   *
   * @param msg the message containing robot state data
   */
  void robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg);

  /**
   * @brief Clears all data in plan member variables
   *
   */
  void clearPlan();

  /**
   * @brief Publish the body plan
   *
   */
  void publishPlan();

  void waitForData();

  /// Subscriber for robot state messages
  ros::Subscriber terrain_map_sub_;

  /// Subscriber for robot state messages
  ros::Subscriber robot_state_sub_;

  /// Subscriber for goal state messages
  ros::Subscriber goal_state_sub_;

  /// Publisher for body plan messages
  ros::Publisher body_plan_pub_;

  /// Publisher for discrete states in body plan messages
  ros::Publisher discrete_body_plan_pub_;

  /// Publisher for the planning tree
  ros::Publisher tree_pub_;

  /// Topic name for terrain map (needed to ensure data has been received)
  std::string terrain_map_topic_;

  /// Topic name for robot state data (needed to ensure data has been received)
  /// // Don't think we need this for offline planner
  std::string robot_state_topic_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data
  double update_rate_;

  /// Handle for the map frame
  std::string map_frame_;

  /// Plan data
  GlobalBodyPlan plan_;

  /// Starting state for planner call
  FullState start_state_;

  /// Goal state for planner
  FullState goal_state_;

  /// Current robot state
  FullState robot_state_;

  /// Starting time for planner call during replans relative to t_plan_[0]
  double replan_start_time_;

  /// goal_state_msg
  geometry_msgs::PointStamped::ConstPtr goal_state_msg_;

  /// Sequence of discrete states in the plan
  std::vector<State> state_sequence_;

  /// Sequence of discrete actions in the plan
  std::vector<Action> action_sequence_;

  /// Planner config
  PlannerConfig planner_config_;

  /// Delay after plan reset before publishing and refining
  double reset_publish_delay_;

  /// Timestep for interpolation
  double dt_;

  /// ID for status of planner
  int planner_status_;

  /// Planning status ID for resetting (start from scratch)
  static const int RESET = 0;

  /// Planning status ID for refining (optimize current plan) // I don't think
  /// we need the status id but for framework we do
  static const int REFINE = 1;

  /// Determines if there is initial plan published
  bool published_plan;
};

#endif  // GLOBAL_BODY_PLANNER_PUBLISHER_H
