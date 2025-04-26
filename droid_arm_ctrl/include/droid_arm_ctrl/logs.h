#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "states_and_cmd.h"

namespace g1_controller {

class Loggers {
 public:
  Loggers(const ros::NodeHandle& handle) : handle_(handle) {
    lowcmd_publisher_ =
        handle_.advertise<sensor_msgs::JointState>(kCmdTopic_, 1);
    lowstate_publisher_ =
        handle_.advertise<sensor_msgs::JointState>(kStateTopic_, 1);
    // Initialize the command message
    cmd_msg_.name = kJointNames_;
    cmd_msg_.position.assign(kJointNames_.size(), 0);
    cmd_msg_.velocity.assign(kJointNames_.size(), 0);
    cmd_msg_.effort.assign(kJointNames_.size(), 0);
    state_msg_.name = kJointNames_;
    state_msg_.position.assign(kJointNames_.size(), 0);
    state_msg_.velocity.assign(kJointNames_.size(), 0);
    state_msg_.effort.assign(kJointNames_.size(), 0);
  }
  ~Loggers() {}

  void publishCmd(const sdk::G1DualArmLowCmd& cmd) {
    cmd_msg_.header.stamp = ros::Time::now();
    for (int i{0}; i < kG1ArmDof_; ++i) {
      cmd_msg_.position[i] = cmd.left_arm.q[i];
      cmd_msg_.velocity[i] = cmd.left_arm.dq[i];
      cmd_msg_.effort[i] = cmd.left_arm.tau[i];
      cmd_msg_.position[i + kG1ArmDof_] = cmd.right_arm.q[i];
      cmd_msg_.velocity[i + kG1ArmDof_] = cmd.right_arm.dq[i];
      cmd_msg_.effort[i + kG1ArmDof_] = cmd.right_arm.tau[i];
    }
    lowcmd_publisher_.publish(cmd_msg_);
  }
  void publishState(const sdk::G1DualArmLowState& state) {
    state_msg_.header.stamp = ros::Time::now();
    for (int i{0}; i < kG1ArmDof_; ++i) {
      state_msg_.position[i] = state.left_arm.q[i];
      state_msg_.velocity[i] = state.left_arm.dq[i];
      state_msg_.effort[i] = state.left_arm.tau[i];
      state_msg_.position[i + kG1ArmDof_] = state.right_arm.q[i];
      state_msg_.velocity[i + kG1ArmDof_] = state.right_arm.dq[i];
      state_msg_.effort[i + kG1ArmDof_] = state.right_arm.tau[i];
    }
    lowstate_publisher_.publish(state_msg_);
  }

 protected:
 protected:
  ros::NodeHandle handle_;
  uint8_t mode_machine_{0};
  ros::Publisher lowcmd_publisher_, lowstate_publisher_;

  const std::string kCmdTopic_{"/g1_arm/cmd"};
  const std::string kStateTopic_{"/g1_arm/lowstate"};
  const std::vector<std::string> kJointNames_{
      "L_SHOULDER_PITCH", "L_SHOULDER_ROLaL", "L_SHOULDER_YAW",
      "L_ELBOW",          "L_WRIST_ROLL",     "L_WRIST_PITCH",
      "L_WRIST_YAW",      "R_SHOULDER_PITCH", "R_SHOULDER_ROLL",
      "R_SHOULDER_YAW",   "R_ELBOW",          "R_WRIST_ROLL",
      "R_WRIST_PITCH",    "R_WRIST_YAW"};
  sensor_msgs::JointState cmd_msg_{}, state_msg_{};
  const int kG1ArmDof_{7};
};
}  // namespace g1_controller
