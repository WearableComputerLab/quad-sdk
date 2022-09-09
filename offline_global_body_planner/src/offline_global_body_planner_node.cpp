#include <iostream>

// #include "offline_global_body_planner/offline_global_body_planner.h"
#include "offline_global_body_planner/offline_global_body_planner_2.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "offline_global_body_planner_2");
  ros::NodeHandle nh;

  /*
  OfflineGlobalBodyPlanner offline_global_body_planner(nh);
  offline_global_body_planner.spin();
  */

  OfflineGlobalBodyPlanner2 offline_global_body_planner2(nh);
  offline_global_body_planner2.spin();
  // Load plan
  /**
  offline_global_body_planner.loadCSVData("/home/andrewzheng1011/quad_sdk_ws/src/quad-sdk/offline_global_body_planner/data/states_traj.csv",
                                          "/home/andrewzheng1011/quad_sdk_ws/src/quad-sdk/offline_global_body_planner/data/ctrl_traj.csv",
                                          4, 2);
  **/
  // Interpolate state action pair

  return 0;
}
