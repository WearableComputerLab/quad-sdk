#include "robot_driver/robot_driver_utils.h"

namespace robot_driver_utils {

void loadMotorCommandMsg(double pos_setpoint, double vel_setpoint, double ff,
                         double kp, double kd, quad_msgs::MotorCommand &msg) {
  // Load commands
  // std::cout << "pos setpoint: " << pos_setpoint << "\t vel setpoint: " << vel_setpoint << std::endl;
  // std::cout << "Torque ff: " << ff << std::endl;
  msg.pos_setpoint = pos_setpoint;
  msg.vel_setpoint = vel_setpoint;
  msg.torque_ff = ff;
  msg.kp = kp;
  msg.kd = kd;
}

}  // namespace robot_driver_utils
