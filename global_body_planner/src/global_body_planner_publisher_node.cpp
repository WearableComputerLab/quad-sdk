#include <ros/ros.h>

// #include "global_body_planner/global_body_planner.h"
#include "global_body_planner/global_body_planner_publisher.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_body_planner_publisher");
  ros::NodeHandle nh;
  GlobalBodyPlannerPublisher global_body_planner(nh);
  global_body_planner.spin();

  // std::cout << "Hello world from main" << std::endl;

  // printHelloWorld();
  return 0;
}
