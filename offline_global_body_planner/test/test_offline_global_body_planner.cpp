#include <gtest/gtest.h>
#include <ros/ros.h>

#include "offline_global_body_planner/offline_global_body_planner_2.h"

// Declare a test
TEST(OfflineGlobalBodyPlannerTest, testOfflineGlobalBodyPlanner2) {
    ros::NodeHandle nh;
    OfflineGlobalBodyPlanner2 offline_global_body_planner_2(nh); // No need to do anything further as warning in loading params built in
    // EXPECT_EQ(1 + 1, 2); // Not sure if you always need this
    // For TTD, offline_global_body_plan should have been test driven code to work way up to offline_global_body_plan_2
    // No more need for complicated exception handling unless reading file and returning boolean (not necessary since ROS_WARN) OR
    // Need Handling exception for when global plan off the map

    // Test case code to handle plan off map here
    // Test case code to handle expected value that plan is inside map

    // Testing:
    // Exception handling and expected value
}

// Run all the tests that were declared with TEST()

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "offline_global_body_planner_2_tester");

    return RUN_ALL_TESTS();
}