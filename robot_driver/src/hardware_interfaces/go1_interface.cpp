#include "robot_driver/hardware_interfaces/go1_interface.h"

using namespace UNITREE_LEGGED_SDK;

Go1Interface::Go1Interface() {}

void Go1Interface::loadInterface(int argc, char** argv) {

  // lowState subscriber runs in its own thread
  thread state_sub_thread(&Go1Interface::start_subscriber, this);
  state_sub_thread.detach();

  ros::NodeHandle nm;
  cmd_pub = nm.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);

  // Set initial lowCmd message configuration
  low_cmd_ros.head[0] = 0xFE;
  low_cmd_ros.head[1] = 0xEF;
  low_cmd_ros.levelFlag = LOWLEVEL;

  for (int i = 0; i < 12; i++) {
    low_cmd_ros.motorCmd[i].mode = 0x0A; // motor switch to servo (PMSM) mode
  }

  // Give the State Sub time to recieve state information
  sleep(3);

  // Initial Message to set the mode of the robot.
  cmd_pub.publish(low_cmd_ros);

  ros::spinOnce();
}

void Go1Interface::unloadInterface() { 
  // Do nothing
}

bool Go1Interface::send(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg,
    const Eigen::VectorXd& user_tx_data) {

    // for each leg
    for (int i = 0; i < 4; ++i) {  // For each leg

      // Order: FL, BL, FR, BR
      quad_msgs::LegCommand leg_command = last_leg_command_array_msg.leg_commands.at(i);

      for (int j = 0; j < 3; ++j) {  // For each joint
      
        int joint_index = (4 * i) + j;
        int unitree_joint_index = joint_indices_to_quad_sdk[joint_indices_to_unitree[joint_index]];

        std::cout << "joint = " << j << std::endl;
        std::cout << std::endl << joint_names[joint_index] << std::endl;
        std::cout << "q: " << leg_command.motor_commands.at(j).pos_setpoint << std::endl;
        std::cout << "dq: " << leg_command.motor_commands.at(j).vel_setpoint << std::endl;
        std::cout << "tau: " << leg_command.motor_commands.at(j).torque_ff << std::endl;
        std::cout << "Kp: " << leg_command.motor_commands.at(j).kp << std::endl;
        std::cout << "Kd: " << leg_command.motor_commands.at(j).kd << std::endl;

        /// Need to ensure the command parameters are correct and safe before sending to real robot.
        // low_cmd_ros.motorCmd[unitree_joint_index].q = leg_command.motor_commands.at(j).pos_setpoint;
        // low_cmd_ros.motorCmd[unitree_joint_index].dq = leg_command.motor_commands.at(j).vel_setpoint;
        // low_cmd_ros.motorCmd[unitree_joint_index].tau = leg_command.motor_commands.at(j).torque_ff;
        // low_cmd_ros.motorCmd[unitree_joint_index].Kp = leg_command.motor_commands.at(j).kp;
        // low_cmd_ros.motorCmd[unitree_joint_index].Kd = leg_command.motor_commands.at(j).kd;
      }
    }

    cmd_pub.publish(low_cmd_ros);

  return true;
}

bool Go1Interface::recv(sensor_msgs::JointState& joint_state_msg,
                           sensor_msgs::Imu& imu_msg,
                           Eigen::VectorXd& user_rx_data) {

  for (int i = 0; i < joint_indices_to_quad_sdk.size(); i++) {
    std::cout << std::to_string(joint_indices_to_quad_sdk[i]) << std::endl;
    std::cout << "q: " << current_low_state.motorState[i].q << std::endl;
    std::cout << "dq: " <<  current_low_state.motorState[i].dq << std::endl;
    std::cout << "tau: " <<  current_low_state.motorState[i].tauEst << std::endl;
  }
  
  std::cout << "quarternion x: " << current_low_state.imu.quaternion[1] << std::endl; // x
  std::cout << "quarternion y " << current_low_state.imu.quaternion[2] << std::endl; // y
  std::cout << "quarternion z: " << current_low_state.imu.quaternion[3] << std::endl; // z
  std::cout << "quarternion w: " << current_low_state.imu.quaternion[0] << std::endl; // w

  std::cout << "gyroscope x: " << current_low_state.imu.gyroscope[0] << std::endl;
  std::cout << "gyroscope y: " << current_low_state.imu.gyroscope[1] << std::endl;
  std::cout << "gyroscope z: " << current_low_state.imu.gyroscope[2] << std::endl;

  // Add the data corresponding to each joint
  for (int i = 0; i < joint_indices_to_quad_sdk.size(); i++) {
    joint_state_msg.name[i] = std::to_string(joint_indices_to_quad_sdk[i]);
    joint_state_msg.position[joint_indices_to_quad_sdk[i]] = current_low_state.motorState[i].q;
    joint_state_msg.velocity[joint_indices_to_quad_sdk[i]] = current_low_state.motorState[i].dq;
    joint_state_msg.effort[joint_indices_to_quad_sdk[i]] = current_low_state.motorState[i].tauEst;
  }
  
  // Translate from normalised unitree imu.quanternion to quaternion
  geometry_msgs::Quaternion orientation_msg;
  orientation_msg.x = current_low_state.imu.quaternion[1]; // x
  orientation_msg.y = current_low_state.imu.quaternion[2]; // y
  orientation_msg.z = current_low_state.imu.quaternion[3]; // z
  orientation_msg.w = current_low_state.imu.quaternion[0]; // w

  // Load the data into the imu message
  imu_msg.orientation = orientation_msg;
  imu_msg.angular_velocity.x = current_low_state.imu.gyroscope[0];
  imu_msg.angular_velocity.y = current_low_state.imu.gyroscope[1];
  imu_msg.angular_velocity.z = current_low_state.imu.gyroscope[2];

  return true;
}

void Go1Interface::start_subscriber() {
  ros::NodeHandle nm; 
  ros::Subscriber low_sub; /** A ros subscriber to listen for state changes from udp_ros*/

  std::cout << "Initializing State Subscriber for Unitree Go1" << std::endl;
  low_sub = nm.subscribe("low_state", 1, &Go1Interface::lowStateCallback, this);
  while(ros::ok()){
    ros::spinOnce();
  }
}

void Go1Interface::lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg)
{
  for (int i(0); i < 20; i++)
  {
      current_low_state.motorState[i] = msg->motorState[i];
  }

  current_low_state.imu = msg->imu;
}