#include <iostream>

#include "offline_global_body_planner/offline_global_body_planner.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offline_global_body_planner");
    ros::NodeHandle nh;

    std::cout << "Hello World from offline_global_body_planner_node!" << std::endl;
    OfflineGlobalBodyPlanner offline_global_body_planner(nh);
    offline_global_body_planner.spin();
    // Load plan
    /**
    offline_global_body_planner.loadCSVData("/home/andrewzheng1011/quad_sdk_ws/src/quad-sdk/offline_global_body_planner/data/states_traj.csv",
                                            "/home/andrewzheng1011/quad_sdk_ws/src/quad-sdk/offline_global_body_planner/data/ctrl_traj.csv",
                                            4, 2);
    **/
    // Interpolate state action pair
    
    return 0;
}