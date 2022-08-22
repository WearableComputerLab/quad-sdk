#ifndef OFFLINE_PLANNING_UTILS_H
#define OFFLINE_PLANNING_UTILS_H

#include <math.h>
#include <quad_utils/fast_terrain_map.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>


#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <vector>


namespace offline_planning_utils {

    /**
     * @brief Planner Configuration
     * 
     */
    struct PlannerConfig {
        FastTerrainMap terrain;                 // Terrain in FastTerrainMap format
        grid_map::GridMap terrain_grid_map;     // Terrain in grid_map format

        // Define kinematic constraint paramters
        double h_max; // Maximum hiehgt of a leg base, m (prob don't need)
        double h_min; // Minimum ground clearance of body corners, m
        double h_nom; // Nominal ground clearance of body, m
        double v_max; // Maximum robot velocity, m/s
        
        // Define dynamic parameters
        double mass; // Robot mass, kg
        double g; // Gravity constant, m/s^2

        // Define planning parameters
        double dt;
        Eigen::Vector3d g_vec;

        // Define robot params and declare points used for validity checking (not sure needed here yet)
        double robot_l; // Length of robot body, m
        double robot_w; // Width of robot body, m
        double robot_h; // Vertical distance between leg base and bottom of robot, m
    };

    /**
     * @brief Define state with Eigen data
     * 
     */
    struct State {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
    };

    /**
     * @brief Define full state with Eigen data
     * 
     */
    struct FullState {
        Eigen::Vector3d pos; // Position
        Eigen::Vector3d vel; // Velocity
        Eigen::Vector3d ang; // Linear Velocity
        Eigen::Vector3d ang_vel; // Angular velocity
    };

    /// Ground reaction force
    typedef Eigen::Vector3d GRF;

    /**
     * @brief Append FullState to State and FullState arrays
     * @param start_state FullState to be appended
     * @param reduced_plan Sequence of States to be appended upon
     * @param dt Time resolution
     * @param full_plan The sequence of FullStates being appended on
     * @param planner_config Configuration parameters
     */
    void addFullStates(const FullState &start_state, const std::vector<State> &reduced_plan, double dt,
                        std::vector<FullState> &full_plan, const PlannerConfig &planner_config);

    /**
     * @brief Convert reduced State to FullState, adding body oroientation and angular speed
     * @param state State
     * @param roll Roll
     * @param pitch Pitch
     * @param yaw Yaw
     * @param roll_rate Rate of change of roll 
     * @param pitch_rate Rate of change of pitch
     * @param yaw_rate Rate of change of yaw
     * @return FullState 
     */
    FullState stateToFullState(const State &state, double roll, double pitch,
                                double yaw, double roll_rate, double pitch_rate, double yaw_rate);

    /**
     * @brief Convert fullstate to state
     * @param full_state FullState
     * @return State 
     */
    State fullStateToState(const FullState &full_state);

    /**
     * @brief Reformat Eigen vector to FullState
     * @param[in] s_eig Eigen vector
     * @param[out] s FullState obtained from Eigen vector
     */
    void eigenToFullState(const Eigen::VectorXd &s_eig, FullState &s);

    /**
     * @brief Reformat FullState to Eigen Vector
     */
    Eigen::VectorXd fullStateToEigen(const FullState &s);

    /**
     * @brief Print State
     * @param s State
     */
    void printState(const State &s);

    /**
     * @brief Print GRF
     * @param grf GRF
     */
    void printGRF(const GRF &grf);

    inline double poseDistance(const State &s1, const State &s2) {
        return (s1.pos - s2.pos).norm();
    }

    /**
     * @brief Print FullState
     * @param s FullState
     */
    void printFullState(const FullState &s);
    


} // namespace planning_utils



#endif