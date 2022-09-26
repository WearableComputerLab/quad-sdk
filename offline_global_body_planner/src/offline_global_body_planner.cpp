#include "offline_global_body_planner/offline_global_body_planner.h"

void printHelloWorld() {
  std::cout << "Hello world from offline_global_body_planner!" << std::endl;
}
// TODO(AZ): Create Plan class s.t. planner and plan are different concepts
// e.g. loadCSVData should be from plan class

OfflineGlobalBodyPlanner::OfflineGlobalBodyPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  std::cout << "OfflineGlobalBodyPlanner Object Initialized" << std::endl;
  // Don't think there is much use of constructor atm
  // Load rosparam from parameter server
  /*
  quad_utils::loadROSParam(nh_, "topics/start_state", robot_state_topic_);
  quad_utils::loadROSParam(nh_, "topics/goal_state", goal_state_topic_);
  quad_utils::loadROSParam(nh_, "/topics/terrain_map", terrain_map_topic_);

  // Setup pubs and subs
  //terrain_map_sub_ = nh_.subscribe(
  //    terrain_map_topic_, 1, &GlobalBodyPlanner::terrainMapCallback, this);
  robot_state_sub_ = nh_.subscribe(
      robot_state_topic_, 1, &OfflineGlobalBodyPlanner::robotStateCallback,
  this);

  */
  body_plan_pub_ =
      nh_.advertise<quad_msgs::RobotPlan>("/robot_1/global_plan", 1);
  discrete_body_plan_pub_ =
      nh_.advertise<quad_msgs::RobotPlan>("/robot_1/global_plan_discrete", 1);

  // Load Planner Configuration
  PlannerConfig planner_config_;
  // Test code (loading and set Misc plan and timestamp should be part of plan
  // class)
  loadCSVData(
      "/home/andrewzheng1011/quad_sdk_ws/src/quad-sdk/"
      "offline_global_body_planner/data/states_traj.csv",
      "/home/andrewzheng1011/quad_sdk_ws/src/quad-sdk/"
      "offline_global_body_planner/data/ctrl_traj.csv",
      4, 2);
  setMiscPlans(0, 0.01, state_sequence_, grf_sequence_, t_plan_,
               primitive_id_plan_, length_plan_, planner_config_);
  setComputedTimestamp(ros::Time::now());

  std::cout << "Length of plan: " << getLengthOfPlan() << std::endl;
  std::cout << "Size of length plan: " << length_plan_.size() << std::endl;
  std::cout << "Size of state_sequence_: " << state_sequence_.size()
            << std::endl;
  std::cout << "Size of action_sequence_: " << grf_sequence_.size()
            << std::endl;  // TODO(AZ): May have to hold pose.. for end action
                           // std::cout << "Full State Plan: " << std::endl;
  // printFullStatePlan(body_plan_);
}

void OfflineGlobalBodyPlanner::setMiscPlans(
    double t0, double dt, std::vector<State> &state_sequence,
    std::vector<GRF> &grf_sequence_, std::vector<double> &t_plan,
    std::vector<int> &primitive_id_plan, std::vector<double> &length_plan,
    const PlannerConfig &planner_config) {
  // TODO(AZ): Integrate w/ planner_config

  // Check validity of dt
  if (dt <= 0) {
    ROS_ERROR("Invalid dt = %0.2f. dt must be greater than 0.", dt);
  }

  int num_discrete_pts = state_sequence.size();
  double plan_length = 0;

  // Configure plans to quad-sdk framework
  for (int i = 0; i < num_discrete_pts; i++) {
    // Add 3rd dim to state & vel (set pos to nominal height at pos of terrain &
    // vel to 0)
    OfflineGlobalBodyPlanner::getNominalHeight(state_sequence.at(i).pos);
    state_sequence.at(i).vel[2] = 0;

    // Add 3rd dim to inputs (i.e. fixed mg)
    OfflineGlobalBodyPlanner::addMG(grf_sequence_.at(i));

    // Fill in time data
    t_plan.push_back(t0 + i * dt);

    // Fill in hardcoded connect primitive id (e.g. 0 == CONNECT == STANCE)
    primitive_id_plan.push_back(0);

    // Fill in length of plan at each index
    if (i == 0) {
      length_plan.push_back(plan_length);
    } else {
      plan_length = plan_length + poseDistance(state_sequence.at(i - 1),
                                               state_sequence.at(i));
      length_plan.push_back(plan_length);
    }
  }

  // Lift from reduced plan into full body plan
  State start_state = state_sequence.at(
      0);  // TODO(AZ): Start state should be obtain from gazebo sim
  FullState start_state_ =
      stateToFullState(start_state, 0, M_PI + 14 * 180 / M_PI, 0, 0, 0,
                       0);  // TODO(AZ): Get from gazebo

  addFullStates(start_state_, state_sequence, 0.01, body_plan_, planner_config);
}

void OfflineGlobalBodyPlanner::printStateSequence(
    const std::vector<State> &state_sequence) {
  for (size_t i = 0; i != state_sequence.size(); i++) {
    std::cout << "Index: " << i << " ";
    printState(state_sequence.at(i));
  }
}

void OfflineGlobalBodyPlanner::printGRFSequence(
    const std::vector<GRF> &grf_sequence) {
  for (size_t i = 0; i != grf_sequence.size(); i++) {
    std::cout << "Index: " << i << " ";
    printGRF(grf_sequence.at(i));
  }
}

void OfflineGlobalBodyPlanner::printFullStatePlan(
    const std::vector<FullState> &full_state_plan) {
  for (size_t i = 0; i != full_state_plan.size(); i++) {
    std::cout << "Index: " << i << std::endl;
    printFullState(full_state_plan.at(i));
  }
}

void OfflineGlobalBodyPlanner::getNominalHeight(
    Eigen::Vector3d &pos) {  // May want to make these functions inline
  pos(2) = 0.25;
}

void OfflineGlobalBodyPlanner::getTerrainZFilteredFromState(
    const Eigen::Vector3d &pos, const PlannerConfig &planner_config) {
  // TODO(AZ): Get pos from fastTerrainMap which is w/in planner_config
  // rn setting terrain height to be hardcoded value (scaling up get from
  // rosparam)
  std::cout << "Hello getTerrainZFilteredFromState" << std::endl;
}

void OfflineGlobalBodyPlanner::addMG(GRF &grf) {
  grf(2) = 12.5 * 9.81;  // TODO(AZ): Get from param server **
}

double OfflineGlobalBodyPlanner::getLengthOfPlan() {
  return length_plan_.at(length_plan_.size() - 1);
}

void OfflineGlobalBodyPlanner::addStateAndGRFToMsg(double t, int plan_index,
                                                   const FullState &body_state,
                                                   const GRF &grf,
                                                   int primitive_id,
                                                   quad_msgs::RobotPlan &msg) {
  // Represent each state as an Odometry message
  quad_msgs::RobotState state;
  // std::cout << "msg.global_plan_timestamp: " <<
  // msg.global_plan_timestamp.toSec() << std::endl;
  quad_utils::updateStateHeaders(state,
                                 msg.global_plan_timestamp + ros::Duration(t),
                                 msg.header.frame_id, plan_index);

  // Load the data into the message
  state.body = quad_utils::eigenToBodyStateMsg(fullStateToEigen(body_state));

  quad_msgs::GRFArray grf_msg;
  geometry_msgs::Vector3 vector_msg;
  vector_msg.x = grf[0];
  vector_msg.y = grf[1];
  vector_msg.z = grf[2];
  geometry_msgs::Point point_msg;
  quad_utils::Eigen3ToPointMsg(body_state.pos, point_msg);

  grf_msg.header = state.header;
  grf_msg.vectors.push_back(vector_msg);
  grf_msg.points.push_back(point_msg);

  bool contact_state =
      (primitive_id !=
       2);  // TODO(AZ): Hardcoded for now... not sure what to do about this
  grf_msg.contact_states.push_back(contact_state);

  msg.states.push_back(state);
  msg.grfs.push_back(grf_msg);
  msg.plan_indices.push_back(plan_index);
  msg.primitive_ids.push_back(primitive_id);
}

void OfflineGlobalBodyPlanner::convertPlanToMsg(
    quad_msgs::RobotPlan &robot_plan_msg,
    quad_msgs::RobotPlan &discrete_robot_plan_msg) {
  // Get body plan size
  int num_discrete_pts = t_plan_.size();

  // Loop through body plan and add to message
  for (std::size_t i = 0; i != num_discrete_pts; i++) {
    addStateAndGRFToMsg(t_plan_[i], i, body_plan_[i], grf_sequence_[i],
                        primitive_id_plan_[i], robot_plan_msg);
  }
  // Loop through discrete states and add to message

  for (std::size_t i = 0; i != state_sequence_.size(); i++) {
    FullState full_discrete_state =
        stateToFullState(state_sequence_[i], 0, 0, 0, 0, 0, 0);
    addStateAndGRFToMsg(0.0, 0, full_discrete_state, grf_sequence_[i],
                        primitive_id_plan_[i], discrete_robot_plan_msg);
  }

  if (robot_plan_msg.states.size() != robot_plan_msg.grfs.size()) {
    throw std::runtime_error(
        "Mismatch between number of states and wrenches, something is wrong");
  }
}

void OfflineGlobalBodyPlanner::terrainMapCallback(
    const grid_map_msgs::GridMap::ConstPtr &msg) {
  std::cout << "Hello terrain map callback" << std::endl;
  // Not being used atm **
  // Need to figure out what to do w/ planner_config_
  /*
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  planner_config_.terrain.loadDataFromGridMap(map);  // Takes ~10ms
  planner_config_.terrain_grid_map = map;            // Takes ~0.1ms
  */
}

void OfflineGlobalBodyPlanner::robotStateCallback(
    // Prob not being used (yet) **
    const quad_msgs::RobotState::ConstPtr &msg) {
  eigenToFullState(quad_utils::bodyStateMsgToEigen(msg->body), robot_state_);
}

void OfflineGlobalBodyPlanner::loadCSVData(std::string state_file_path,
                                           std::string ctrl_file_path,
                                           int num_states, int num_ctrl) {
  // Code intended to be upscale for 3D spatial case
  // Please generalize it if code is poorly written from that framework

  std::cout << "Hello from loadCSVData" << std::endl;
  const int n = num_states;  // Dimension of states
  const int spatial_dim =
      n / 2;  // The dimension of the spatial space (e.g. 2D) (Half of the full
              // state dim) (Hard stop at 3)
  const int ctrl_dim = num_ctrl;  // Dimension of control (e.g. u_x & u_y)

  // File pointer
  std::ifstream stateInputFile;
  std::ifstream controlInputFile;

  std::vector<std::vector<double> > states_traj;
  std::vector<std::vector<double> > ctrls_traj;

  stateInputFile.open(state_file_path);
  controlInputFile.open(ctrl_file_path);

  if (stateInputFile.fail() || controlInputFile.fail()) {
    ROS_ERROR("Failed to open state or control trajectory csv file");
    ROS_ERROR("File paths: \n%s\n%s", state_file_path.c_str(),
              ctrl_file_path.c_str());
  }

  // Load data into stl vector of type State and GRF
  std::string state_line,
      ctrl_line;  // String object to read each line of state and ctrl (Can
                  // technically use just one line)
  State temp_state;
  GRF temp_ctrl;

  int counter = 0;
  while (std::getline(stateInputFile, state_line)) {
    counter++;
    // Read each line (i.e. All states at each index)
    // std::cout << "counter: " << counter << std::endl; // For debugging
    // purpose

    Eigen::Vector3d holder = Eigen::Vector3d::Zero();

    // Read each string entry in line and convert to double
    std::stringstream ss(state_line);
    std::string data;
    for (int i = 0; i < spatial_dim; i++) {
      std::getline(ss, data, ',');
      holder(i) = std::stod(data);
    }
    temp_state.pos = holder;
    holder = Eigen::Vector3d::Zero();  // Reset

    for (int j = 0; j < n - spatial_dim; j++) {
      std::getline(ss, data, ',');
      holder(j) = std::stod(data);
    }
    temp_state.vel = holder;
    state_sequence_.push_back(temp_state);
  }

  while (std::getline(controlInputFile, ctrl_line)) {
    counter++;
    // Read each line (i.e. All ctrl at each index)
    // std::cout << "counter: " << counter << std::endl; // For debugging
    // purpose

    Eigen::Vector3d holder = Eigen::Vector3d::Zero();

    // Read each string entry in line and convert to double
    std::stringstream ss(ctrl_line);
    std::string data;
    for (int i = 0; i < ctrl_dim; i++) {
      std::getline(ss, data, ',');
      holder(i) = std::stod(data);
    }
    temp_ctrl = holder;
    // std::cout << "ctrl: \n" << temp_ctrl << std::endl;
    grf_sequence_.push_back(temp_ctrl);
  }
}

void OfflineGlobalBodyPlanner::publishPlan() {
  // Declare the messages for FullState body plan and FullState discrete plan
  quad_msgs::RobotPlan robot_plan_msg;
  quad_msgs::RobotPlan discrete_robot_plan_msg;

  // Need sleep for some reason
  ros::Duration(2.0)
      .sleep();  // Sleep to make rostopic echo line up with nmpc timestamp
  robot_plan_msg.header.frame_id = "map";
  robot_plan_msg.header.stamp =
      ros::Time::now();  // Initialize timestamp for msg (This if finicky)

  // If first and only plan has not been published, set published timestamp for
  // plan
  if (!published_plan_) {
    // Initializing timestamp for your plan
    // plan_.setPublishedTimestamp(ros::Time::now());
    published_timestamp_ = ros::Time::now();
    // ROS_WARN("Published timestamp: %0.2f", published_timestamp_.toSec());
  }

  discrete_robot_plan_msg.header = robot_plan_msg.header;
  // Initialize the headers and types
  robot_plan_msg.global_plan_timestamp = published_timestamp_;
  discrete_robot_plan_msg.global_plan_timestamp = published_timestamp_;

  // Load the plan into msg
  convertPlanToMsg(robot_plan_msg, discrete_robot_plan_msg);

  // Publish both messages
  body_plan_pub_.publish(robot_plan_msg);
  discrete_body_plan_pub_.publish(discrete_robot_plan_msg);

  if (!published_plan_) {
    ROS_WARN("First plan published, stamp = %0.5f",
             robot_plan_msg.global_plan_timestamp.toSec());
    published_plan_ = true;
  }
}

void OfflineGlobalBodyPlanner::spin() {
  // ros::Rate r(update_rate_);
  ros::Rate r(20);

  // Should do readCSVData and setMiscPlan as callPlanner

  while (ros::ok()) {
    // Process callbacks
    ros::spinOnce();
    publishPlan();
    // ROS_INFO("Publishing plan...");

    r.sleep();
  }
}
