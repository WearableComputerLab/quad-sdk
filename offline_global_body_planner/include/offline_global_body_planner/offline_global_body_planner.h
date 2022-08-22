#ifndef OFFLINE_GLOBAL_BODY_PLANNER_H
#define OFFLINE_GLOBAL_BODY_PLANNER_H


#include <fstream>
#include <sstream>

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

#include "offline_global_body_planner/offline_planning_utils.h"

// #include "global_body_planner/gbpl.h" // Don't think I need this
// #include "global_body_planner/global_body_plan.h" // Probably also do not need this


using namespace offline_planning_utils;


//! An offline global body planner for legged robots
/*!
    OfflineGlobalBodyPlanner contains all the logic contained in the OfflineGlobalBodyPlanner
    node. Currently, it's main purpose is to generally take a trajectory of states and control
    inputs and publish it in a RobotPlan msg format.
*/
class OfflineGlobalBodyPlanner {
   public:

    /**
     * @brief Constructor for Offline Global Body Planner class
     * @param nh Node Handle
     * @return Constructed object of type OfflineGlobalBodyPlanner
     */
    OfflineGlobalBodyPlanner(ros::NodeHandle nh);

    /**
     * @brief Reads in filePath trajectory and defines state and control
     * trajectory
     * @param stateFilePath csv filepath to state trajectory
     * @param controlFilePath csv filepath to control
     * @param numStates Dimension of states
     * @param numCtrl Dimension of control
     */
    void loadCSVData(std::string state_file_path, std::string ctrl_file_path, int num_states, int num_ctrl);

    /**
     * @brief Publish FullState plan
     */
    void publishPlan();  

    /**
     * @brief Get the position of quadruped given height and location w/in terrain
     * @param pos State position
     * @param planner_config Planner configuration
     */
    void getNominalHeight(Eigen::Vector3d &pos); // Add planner_config as parameter later


    /**
     * @brief Add mg to GRF due to planar planner
     * @param grf Ground Reaction force added with fixed z component
     */
    void addMG(GRF &grf);

    /**
     * @brief Add fixed primitive ID into plan (Remove in future potentially - AZ)
     */
    void addWalkID();

    /**
     * @brief Get dist to goal from control and state trajectory
     * 
     */
    void getDistToGoal();

    /**
     * @brief Print state_sequence
     * @param state_sequence Sequence of state
     */
    void printStateSequence(const std::vector<State> &state_sequence);

    /**
     * @brief Print grf_sequence
     * @param grf_sequence Sequence of action
     */
    void printGRFSequence(const std::vector<GRF> &grf_sequence);

    /**
     * @brief Print FullState plan
     * @param full_state_plan FullState plan
     */
    void printFullStatePlan(const std::vector<FullState> &full_state_plan);

    /**
     * @brief 
     * @param t 
     * @param plan_index 
     * @param body_state 
     * @param grf 
     * @param primitive_id 
     * @param msg 
     */
    void addStateAndGRFToMsg(double t, int plan_index,
                            const FullState &body_state, const GRF &grf,
                            int primitive_id, quad_msgs::RobotPlan &msg);
    /**
     * @brief Set the timestamp at which the plan was computed (unique)
     * @param timestamp Timestamp at which plan returned
     */
    inline void setComputedTimestamp(ros::Time timestamp) {
        computed_timestamp_ = timestamp;
    }

    /**
     * @brief Set the timestamp at which the plan was published
     * @param timestamp Timestamp at which plan was published
     */
    inline void setPublishedTimestamp(ros::Time timestamp) {
        published_timestamp_ = timestamp;
    }

    /**
     * @brief Get the Computed Timestamp
     * @return ros::Time Timestamp at which plan computed
     */
    inline ros::Time getComputedTimestamp() const { return computed_timestamp_; }

    /**
     * @brief Get the Published Timestamp
     * @return ros::Time Timestamp at which a plan was published
     */
    inline ros::Time getPublishedTimestamp() const { return published_timestamp_; }


    /**
     * @brief Primary work function in class, called in node file for this component
     */
    void spin();



   private:
    /**
     * @brief Get the Terrain Z Filtered From State object
     * 
     * @param pos State position
     * @param planner_config Configuration parameters
     */
    void getTerrainZFilteredFromState(const Eigen::Vector3d &pos, const PlannerConfig &planner_config);

    /**
     * @brief Callback function to handle new terrain map data
     * @param msg the message containing map data
     */

    void terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr &msg);

    /**
    * @brief Callback function to handle new robot state data
    * @param[in] msg the message contining robot state data
    */
    void robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg);

    /**
     * @brief Set additional parameters to be viable for quad-sdk framework
     * 
     */
    void setMiscPlans(double t0, double dt, std::vector<State> &state_sequence, std::vector<GRF> &grf_plan,
                    std::vector<double> &t_plan, std::vector<int> &primitive_id_plan, std::vector<double> &length_plan,
                    const PlannerConfig &planner_config);
    /**
     * @brief Convert reduced double integrator states to FullState
     * 
     */
    void addFullState(); // Probably want my own struct FullState later

    /**
     * @brief Resets the body plan to 0
     * 
     */
    void resetBodyPlan();

    /**
     * @brief Resets the state and action sequence to 0
     * 
     */
    void resetSequence();

    /**
     * @brief Get the Length Of Plan
     * @return double The distance [m] of the plan
     */
    double getLengthOfPlan();

    /**
     * @brief Loads all body plan into a RobotPlan msg
     * @param robot_plan_msg Robot plan message
     * @param discrete_robot_plan_msg Discrete robot plan message
     */
    void convertPlanToMsg(quad_msgs::RobotPlan &robot_plan_msg, quad_msgs::RobotPlan &discrete_robot_plan_msg);


    /// Node Handle
    ros::NodeHandle nh_;

    // Variables that seem to go into plan need to be moved into plan class in future ** TODO
    /// Time stamp for when plan was computed/loaded in this case (unique)
    ros::Time computed_timestamp_;

    /// Time stamp for when plan was published (not unique)
    ros::Time published_timestamp_;

    /// Std vector containing the robot body plan
    std::vector<FullState> body_plan_;

    /// Std vector containing the discrete states in the sequence (not quite a plan yet.. feel free to name better)
    std::vector<State> state_sequence_;

    /// Std vector containing the GRF in the plan
    std::vector<GRF> grf_sequence_;

    /// Std vector containing primitive id plan
    std::vector<int> primitive_id_plan_;

    /// Std vector containing time data
    std::vector<double> t_plan_;

    /// Std vector containing cimulative path length at each index of the plan
    std::vector<double> length_plan_;

    /// Subscriber for terrain map messages
    ros::Subscriber terrain_map_sub_;

    /// Subscriber for robot state messages
    ros::Subscriber robot_state_sub_;

    /// Subscriber for goal messages
    ros::Subscriber goal_state_sub_;

    /// Publisher for body plan messages
    ros::Subscriber body_plan_sub_;

    /// Publisher for body plan messages
    ros::Publisher body_plan_pub_;

    /// Publisher for discrete states in body plan messages
    ros::Publisher discrete_body_plan_pub_;

    /// Publisher for the planning tree
    ros::Publisher tree_pub_; // Prob don't need

    /// Topic name for terrain map
    std::string terrain_map_topic_;

    /// Topic name for robot state data
    std::string robot_state_topic_;

    /// Topic name for goal state topic
    std::string goal_state_topic_;

    /// Handle for the map frame
    std::string map_frame_;

    /// Starting state for planner call
    FullState start_state_;

    /// Goal state for planner
    FullState goal_state_;

    /// Current robot state
    FullState robot_state_;

    /// Goal state msg
    geometry_msgs::PointStamped::ConstPtr goal_state_msg_;

    // Published plan
    bool published_plan_;

    
};

void printHelloWorld();

#endif // OFFLINE_GLOBAL_BODY_PLANNER_H