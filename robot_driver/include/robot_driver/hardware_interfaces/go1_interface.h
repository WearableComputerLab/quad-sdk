#ifndef GO1_INTERFACE_H
#define GO1_INTERFACE_H

#include <thread>
#include <quad_msgs/LegCommandArray.h>
#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>


//! Hardware interface for the Go1 EDU quadruped from Unitree Robotics.
/*!
   Go1Interface listens for joint control messages and outputs low level
   commands to the real quadruped over UDP.

   Ensure that the robot is connected via ethernet and that your Ethernet Adapter
   has been configured to 192.168.123.162 as per Unitree instruction.
*/
class Go1Interface : public HardwareInterface {
 public:
  /**
   * @brief Constructor for Go1Interface
   * @return Constructed object of type Go1Interface
   */
  Go1Interface();

  /**
   * @brief Load the hardware interface
   * @param[in] argc Argument count
   * @param[in] argv Argument vector
   */
  virtual void loadInterface(int argc, char** argv);

  /**
   * @brief Unload the hardware interfacee
   */
  virtual void unloadInterface();

  /**
   * @brief Send commands to the robot via the mblink protocol
   * @param[in] leg_command_array_msg Message containing leg commands
   * @param[in] user_data Vector containing user data
   * @return boolean indicating success of transmission
   */
  virtual bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
                    const Eigen::VectorXd& user_tx_data);

  /**
   * @brief Recieve data from the robot via the mblink protocol
   * @param[out] joint_state_msg Message containing joint state information
   * @param[out] imu_msg Message containing imu information
   * @param[out] user_data Vector containing user data
   * @return Boolean for whether data was successfully received
   */
  virtual bool recv(sensor_msgs::JointState& joint_state_msg,
                    sensor_msgs::Imu& imu_msg, Eigen::VectorXd& user_rx_data);

  void start_subscriber();

  void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg);

  private:
    ros::Publisher cmd_pub;
    unitree_legged_msgs::LowCmd low_cmd_ros; /** A unitree legged message which contains 12 motor commands for each joint*/
    unitree_legged_msgs::LowState current_low_state;

  /// Vector denoting joint indices
  // limb descriptions unitree                  |--FR-|  |--FL-|  |--RR-|  |--RL--|
  // UNITREE MotorState indicies             = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}                
  std::vector<int> joint_indices_to_quad_sdk = {10, 4, 5, 8, 0, 1, 11, 6, 7, 9, 2, 3};

  // limb descriptions unitree                  |--FL-|  |--RL-|  |--FR-|  |--RR--|
  // UNITREE MotorState indicies             = {8, 0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11}    
  std::vector<int> joint_indices_to_unitree  = {8, 0, 1, 8, 0, 1, 11, 6, 7, 9, 2, 3};
  std::vector<std::string> joint_names = {"FL_HIP", "FL_THIGH", "FL_CALF", "RL_HIP", "RL_THIGH", "RL_CALF","FR_HIP", "FR_THIGH", "FR_CALF", "RR_HIP", "RR_THIGH", "RR_CALF"};
};

#endif  // GO1_INTERFACE_H
