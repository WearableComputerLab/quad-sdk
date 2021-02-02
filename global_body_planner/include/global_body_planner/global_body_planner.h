#ifndef GLOBAL_BODY_PLANNER_H
#define GLOBAL_BODY_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/StateEstimate.h>

#include "global_body_planner/rrt_connect.h"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace planning_utils;

//! A global body planning class for legged robots
/*!
   GlobalBodyPlanner is a container for all of the logic utilized in the global body planning node.
   This algorithm requires an height map of the terrain as a GridMap message type, and will publish
   the global body plan as a BodyPlan message over a topic. It will also publish the discrete states
   used by the planner (from which the full path is interpolated).
*/
class GlobalBodyPlanner {
  public:
    /**
     * @brief Constructor for GlobalBodyPlanner Class
     * @param[in] nh Node handle
     * @return Constructed object of type GlobalBodyPlanner
     */
    GlobalBodyPlanner(ros::NodeHandle nh);

    /**
     * @brief Call the correct planning class and compute statistics
     */
    void callPlanner();

    /**
     * @brief Primary work function in class, called in node file for this component
     */
    void spin();

  private:
    /**
     * @brief Callback function to handle new terrain map data
     * @param[in] msg the message contining map data
     */
    void terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);

    /**
     * @brief Callback function to handle new ground truth state data
     * @param[in] msg the message contining ground truth state data
     */
    void robotStateCallback(const spirit_msgs::StateEstimate::ConstPtr& msg);

    /**
     * @brief Check if a replan is required
     * @return boolean for whether to replan
     */
    bool replanTrigger();

    /**
     * @brief Initialize the planner by clearing out old plan data and setting the start state
     * @return Index of the current plan from which to being the new plan (zero if fully replanning)
     */
    int initPlanner();

    /**
     * @brief Clear all data in plan member variables
     */
    void clearPlan();

    /**
     * @brief Update the body plan with the current plan
     * @param[in] t Time of state in trajectory
     * @param[in] body_state Body state
     * @param[in] body_wrench Wrench applied to body
     * @param[in] body_plan_msg Body plan message
     */
    void addStateWrenchToMsg(double t, FullState body_state, Wrench body_wrench, spirit_msgs::BodyPlan& body_plan_msg);

    /**
     * @brief Publish the current body plan
     */
    void publishPlan();

    /**
     * @brief Wait until map and state messages have been received and processed
     */
    void waitForData();

    /// Subscriber for terrain map messages
    ros::Subscriber terrain_map_sub_;

    /// Subscriber for robot state messages
    ros::Subscriber robot_state_sub_;

    /// Publisher for body plan messages
    ros::Publisher body_plan_pub_;

    /// Publisher for discrete states in body plan messages
    ros::Publisher discrete_body_plan_pub_;

    /// Topic name for terrain map (needed to ensure data has been received)
    std::string terrain_map_topic_;

    /// Topic name for robot state data (needed to ensure data has been received)
    std::string robot_state_topic_;

    /// Nodehandle to pub to and sub from
    ros::NodeHandle nh_;

    /// Update rate for sending and receiving data;
    double update_rate_;

    /// Number of times to call the planner
    int num_calls_;

    /// Max time to let the algorithm search
    double max_time_;

    /// Handle for the map frame
    std::string map_frame_;

    /// Struct for terrain map data
    FastTerrainMap terrain_;

    /// Std vector containing the interpolated robot body plan
    std::vector<FullState> body_plan_;

    /// Std vector containing the interpolated wrench plan
    std::vector<Wrench> wrench_plan_;

    /// Std vector containing the interpolated time data
    std::vector<double> t_plan_;

    /// Starting state for planner
    std::vector<double> start_state_;

    /// Starting time for planner (for replanning)
    double start_time_;

    /// Horizon to commit to (replan from the next state after this horizon)
    double committed_horizon_;

    /// Threshold of state error to trigger replanning
    double state_error_threshold_;

    /// Current robot state
    std::vector<double> robot_state_;

    /// Flag to determine if the planner needs to restart planning from the robot state
    bool plan_from_robot_state_flag_;

    /// Goal state for planner
    std::vector<double> goal_state_;
    
    /// Sequence of discrete states in the plan
    std::vector<State> state_sequence_;

    /// Sequence of discrete actions in the plan
    std::vector<Action> action_sequence_;

    /// Cost of the current path in path length
    double current_cost_;

    /// Vector of cost instances in each planning call
    std::vector<double> cost_vector_;

    /// Vector of time instances of cost data for each planning call
    std::vector<double> cost_vector_times_;

    /// Vector of solve times for each planning call
    std::vector<double> solve_time_info_;

    /// Vector of number of vertices for each planning call
    std::vector<int> vertices_generated_info_;

};


#endif // GLOBAL_BODY_PLANNER_H
