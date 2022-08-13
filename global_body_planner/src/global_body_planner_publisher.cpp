#include "global_body_planner/global_body_planner_publisher.h"


int num_calls;

GlobalBodyPlannerPublisher::GlobalBodyPlannerPublisher(ros::NodeHandle nh){
    ROS_INFO("Successfully initialized object GlobalBodyPlannerPublisher");
    nh_ = nh;

    // Load rosparams from parameter server
    std::string body_plan_topic, discrete_body_plan_topic, body_plan_tree_topic,
    goal_state_topic;
    std::vector<double> goal_state_vec(2);
    
    quad_utils::loadROSParam(nh_, "topics/start_state", robot_state_topic_);
    quad_utils::loadROSParam(nh_, "topics/goal_state", goal_state_topic);
    quad_utils::loadROSParam(nh_, "/topics/terrain_map", terrain_map_topic_);
    quad_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic); // Need to consider way to approach this topic l8r
    quad_utils::loadROSParam(nh_, "topics/global_plan_discrete",
                            discrete_body_plan_topic);
    quad_utils::loadROSParam(nh_, "topics/global_plan_tree",
                            body_plan_tree_topic);
    quad_utils::loadROSParam(nh_, "/map_frame", map_frame_);
    quad_utils::loadROSParam(nh_, "/global_body_planner/update_rate",
                            update_rate_);
    
    // Don't think I need any of this, but maybe startup delay
    /*
    quad_utils::loadROSParam(nh_, "global_body_planner/num_calls", num_calls_);
    quad_utils::loadROSParam(nh_, "global_body_planner/max_planning_time",
                            max_planning_time_);
    quad_utils::loadROSParam(nh_, "global_body_planner/pos_error_threshold",
                            pos_error_threshold_);
    quad_utils::loadROSParam(nh_, "global_body_planner/startup_delay",
                            reset_publish_delay_);
    quad_utils::loadROSParam(nh_, "global_body_planner/replanning",
                            replanning_allowed_);
    */

    quad_utils::loadROSParam(nh_, "/local_planner/timestep", dt_);
    quad_utils::loadROSParam(nh_, "/global_body_planner/goal_state",
                            goal_state_vec);

    // Setup pubs and subs
    
    
    terrain_map_sub_ = nh_.subscribe(
        terrain_map_topic_, 1, &GlobalBodyPlannerPublisher::terrainMapCallback, this);
    
    robot_state_sub_ = nh_.subscribe(
        robot_state_topic_, 1, &GlobalBodyPlannerPublisher::robotStateCallback, this);
    // goal_state_sub_ = nh_.subscribe(goal_state_topic, 1,
    //                                &GlobalBodyPlannerPublisher::goalStateCallback, this);
    body_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(body_plan_topic, 1);
    discrete_body_plan_pub_ =
        nh_.advertise<quad_msgs::RobotPlan>(discrete_body_plan_topic, 1);
    tree_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(body_plan_tree_topic, 1);
    

    // Load planner config
    bool enable_leaping; // Need to set this false if using this framework
    planner_config_.loadParamsFromServer(nh);
    nh_.param<bool>("global_body_planner/enable_leaping", enable_leaping, true);
    if (!enable_leaping) {
        planner_config_.enable_leaping = false;
        planner_config_.num_leap_samples = 0;
        planner_config_.h_min = 0;
        planner_config_.h_max = 0.5;
    }

    // Fill in the goal state information
    goal_state_vec.resize(12, 0);
    vectorToFullState(goal_state_vec, goal_state_);

    // Set plan to not be found
    published_plan = false;
}


void GlobalBodyPlannerPublisher::terrainMapCallback(
    const grid_map_msgs::GridMap::ConstPtr &msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  planner_config_.terrain.loadDataFromGridMap(map);  // Takes ~10ms
  planner_config_.terrain_grid_map = map;            // Takes ~0.1ms

  // Uodate the goal state of the planner
  goal_state_.pos[2] =
      planner_config_.h_nom + planner_config_.terrain.getGroundHeight(
                                  goal_state_.pos[0], goal_state_.pos[1]);
} 


void GlobalBodyPlannerPublisher::robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg) {
    eigenToFullState(quad_utils::bodyStateMsgToEigen(msg->body), robot_state_);
}


void GlobalBodyPlannerPublisher::waitForData() {
    // Spin until terrain map message has been received and processed
    boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
    while ((shared_map == nullptr) && ros::ok()) {
        shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>(
            terrain_map_topic_, nh_);
        ros::spinOnce();
    }

    boost::shared_ptr<quad_msgs::RobotState const> shared_robot_state;
    while ((shared_robot_state == nullptr) && ros::ok()) {
        shared_robot_state = ros::topic::waitForMessage<quad_msgs::RobotState>(
            robot_state_topic_, nh_);
        ros::spinOnce();
    }
    ROS_INFO("GBP has state and map information");
    // reset_time_ = ros::Time::now(); // Don't think I need this variable
}

bool GlobalBodyPlannerPublisher::callPlanner() {
    // Define start state and goal state
    // start_state_ = robot_state_;
    Eigen::Vector3d start_pos, goal_pos, start_vel, goal_vel;
    start_pos << -1.0, 0.0, 0.270;
    goal_pos << 5.0, 1.5, 0.3;
    start_vel << 0.0, 0.0, 0.0;
    goal_vel << 0.0, 0.0, 0.0;

    State start_state, goal_state;
    start_state.pos = start_pos;
    start_state.vel = start_vel; // Can do .setZero()

    goal_state.pos = goal_pos;
    goal_state.vel = goal_vel;

    // Define Action
    Eigen::Vector3d hold_grf_0, hold_grf_f;
    hold_grf_0 << 0.0237608, 0.00597425, 0.271616; // z component will turn to m*g in convertToMsg
    hold_grf_f << -0.0238854, -0.00597481, 0.3;

    Action a_hold;
    a_hold.grf_0 = hold_grf_0;
    a_hold.grf_f = hold_grf_f;
    a_hold.t_s_leap = 12.3925; //12.3925; Try 15.0
    a_hold.t_f = 0.0;
    a_hold.t_s_land = 0.0;
    a_hold.dz_0 = 0.0;
    a_hold.dz_f = 0.0;


    // Push hardcode state and action vectors into action and state sequence
    std::vector<State> state_sequence;
    std::vector<Action> action_sequence;

    state_sequence.push_back(start_state);
    state_sequence.push_back(goal_state);

    action_sequence.push_back(a_hold);

    // Get additional variables for loadPlanData
    double dist_to_goal_ = poseDistance(start_state, goal_state);
    int plan_status = VALID; // VALID := 1 to justify plan is valid
    replan_start_time_ = 0;// Start time may need to be gotten from setComputedTimestamp... do I need this..? maybe

    ros::Time plan_start_timestamp_ = ros::Time::now();

    FullState start_state_ = stateToFullState(start_state, 0, 0, 0, 0, 0, 0); // Convert start state to FullState datatype

    // Set up Global Body Plan
    // std::cout << "Computed timestamp: " << plan_start_timestamp_.toSec() << std::endl;
    // Don't think computedTimestamp is useful... only used as a way to compare if planner is equal to another?
    plan_.setComputedTimestamp(plan_start_timestamp_); // eraseAfterIndex(int n) useful if need to replan
    plan_.loadPlanData(plan_status, start_state_, dist_to_goal_, state_sequence, action_sequence, dt_, replan_start_time_, planner_config_);



}

void GlobalBodyPlannerPublisher::publishPlan() {
    // Declare the messages for interpolated body plan and discrete states,
    // initialize their headers
    quad_msgs::RobotPlan robot_plan_msg;
    quad_msgs::RobotPlan discrete_robot_plan_msg;

    robot_plan_msg.header.frame_id = map_frame_;
    robot_plan_msg.header.stamp = ros::Time::now(); // Initialize timestamp for msg
    
    // If first and only plan has not been published, set published timestamp for plan
    if (!published_plan) {
        // Initializing timestamp for your plan
        plan_.setPublishedTimestamp(ros::Time::now());
    }


    discrete_robot_plan_msg.header = robot_plan_msg.header;


    // Initialize the headers and types 
    robot_plan_msg.global_plan_timestamp = plan_.getPublishedTimestamp();
    discrete_robot_plan_msg.global_plan_timestamp = plan_.getPublishedTimestamp();

    // Load the plan into the messages
    plan_.convertToMsg(robot_plan_msg, discrete_robot_plan_msg);

    // Publish both messages
    body_plan_pub_.publish(robot_plan_msg); // PUBLISH BODY PLAN TO MSG HERE... ALREADY INTERPOLATED HERE IT SEEMS
    discrete_body_plan_pub_.publish(discrete_robot_plan_msg);

    if(!published_plan){
        ROS_WARN("First plan published, stamp = %f",
        robot_plan_msg.global_plan_timestamp.toSec());
        published_plan = true;
    }
}

void GlobalBodyPlannerPublisher::spin() {
    // ros::Rate r(update_rate_);
    ros::Rate r(1);

    // Wait until we get map and state data
    waitForData();
    // Hardcode call planner and publish plan
    // Doesn't seem to work w/o doing spin.. may need a server
    callPlanner();
    //publishPlan();
    
    // Enter main spin
    
    while (ros::ok()) {
        // Process callbacks
        
        ros::spinOnce();
        publishPlan();
        // ROS_INFO("Publishing plan...");
        
        r.sleep();
    } 
}