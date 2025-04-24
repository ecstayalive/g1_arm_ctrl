#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <chrono>
#include <thread>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "droid_arm_ctrl/states_and_cmd.h"

namespace sdk {

class DualArmAPI {
 public:
  virtual ~DualArmAPI() {};
  virtual void send() = 0;
  virtual void recv() = 0;
  void sendRecv() {
    send();
    recv();
  };
  virtual void setCmd(const G1DualArmLowCmd& cmd) = 0;
  virtual void getState(G1DualArmLowState& state) = 0;
};

struct ApiRegistry {
  static void registerApi(const std::string& name, DualArmAPI* api) {
    if (api_map_.find(name) != api_map_.end()) {
      ROS_WARN("Warning: API %s already registered, overwriting", name.c_str());
    }
    api_map_[name] = api;
  }

 private:
  static std::unordered_map<std::string, DualArmAPI*> api_map_;
};

class G1DualArmAPI : public DualArmAPI {
 public:
  G1DualArmAPI(const std::string& interface = "eth0") {
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    lowcmd_publisher_.reset(
        new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
            kArmCmdTopic_));
    lowcmd_publisher_->InitChannel();
    lowstate_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            kStatesTopic_));
    lowstate_subscriber_->InitChannel(
        std::bind(&G1DualArmAPI::lowStateCallback, this, std::placeholders::_1),
        1);
    while (low_state_.tick() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    for (int i{0}; i < low_state_.motor_state().size(); ++i) {
      // low_cmd_.motor_cmd()[i].mode(0);
      low_cmd_.motor_cmd()[i].kp(0.f);
      low_cmd_.motor_cmd()[i].kd(1.f);
      low_cmd_.motor_cmd()[i].q(low_state_.motor_state()[i].q());
      low_cmd_.motor_cmd()[i].dq(0.f);
      low_cmd_.motor_cmd()[i].tau(0.f);
    }
  }
  void send() override { lowcmd_publisher_->Write(low_cmd_); }
  void recv() override {}
  void setCmd(const G1DualArmLowCmd& cmd) override {
    low_cmd_.mode_pr() = mode_;
    low_cmd_.mode_machine() = mode_machine_;
    for (int i{0}; i < kG1ArmIdxStart_; ++i) {
      low_cmd_.motor_cmd()[i].mode(0);
      low_cmd_.motor_cmd()[i].kp(0.f);
      low_cmd_.motor_cmd()[i].kd(0.f);
      low_cmd_.motor_cmd()[i].q(low_state_.motor_state()[i].q());
      low_cmd_.motor_cmd()[i].dq(0.f);
      low_cmd_.motor_cmd()[i].tau(0.f);
    }
    for (int i{kG1ArmIdxStart_}; i < kG1ArmIdxStart_ + kG1ArmDof_; ++i) {
      low_cmd_.motor_cmd()[i].mode(1);
      low_cmd_.motor_cmd()[i + kG1ArmDof_].mode(1);
      if (checkJointSafety(cmd, i)) {
        low_cmd_.motor_cmd()[i].kp(cmd.left_arm.kp[i - kG1ArmIdxStart_]);
        low_cmd_.motor_cmd()[i].kd(cmd.left_arm.kd[i - kG1ArmIdxStart_]);
        low_cmd_.motor_cmd()[i].q(cmd.left_arm.q[i - kG1ArmIdxStart_]);
        low_cmd_.motor_cmd()[i].dq(cmd.left_arm.dq[i - kG1ArmIdxStart_]);
        low_cmd_.motor_cmd()[i].tau(cmd.left_arm.tau[i - kG1ArmIdxStart_]);
      }
      if (checkJointSafety(cmd, i + kG1ArmDof_)) {
        low_cmd_.motor_cmd()[i + kG1ArmDof_].kp(
            cmd.right_arm.kp[i - kG1ArmIdxStart_]);
        low_cmd_.motor_cmd()[i + kG1ArmDof_].kd(
            cmd.right_arm.kd[i - kG1ArmIdxStart_]);
        low_cmd_.motor_cmd()[i + kG1ArmDof_].q(
            cmd.right_arm.q[i - kG1ArmIdxStart_]);
        low_cmd_.motor_cmd()[i + kG1ArmDof_].dq(
            cmd.right_arm.dq[i - kG1ArmIdxStart_]);
        low_cmd_.motor_cmd()[i + kG1ArmDof_].tau(
            cmd.right_arm.tau[i - kG1ArmIdxStart_]);
      }
    }
    low_cmd_.crc() =
        crc32Core((uint32_t*)&low_cmd_, (sizeof(low_cmd_) >> 2) - 1);
  }
  void getState(G1DualArmLowState& state) override {
    for (int i{kG1ArmIdxStart_}; i < kG1ArmIdxStart_ + kG1ArmDof_; ++i) {
      state.left_arm.q[i - kG1ArmIdxStart_] = low_state_.motor_state()[i].q();
      state.left_arm.dq[i - kG1ArmIdxStart_] = low_state_.motor_state()[i].dq();
      state.left_arm.ddq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i].ddq();
      state.left_arm.tau[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i].tau_est();
      state.right_arm.q[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].q();
      state.right_arm.dq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].dq();
      state.right_arm.ddq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].ddq();
      state.right_arm.tau[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].tau_est();
    }
    state.tick = low_state_.tick();
  }

 private:
  uint32_t crc32Core(const uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
      xbit = 1 << 31;
      data = ptr[i];
      for (uint32_t bits = 0; bits < 32; bits++) {
        if (CRC32 & 0x80000000) {
          CRC32 <<= 1;
          CRC32 ^= dwPolynomial;
        } else
          CRC32 <<= 1;
        if (data & xbit) CRC32 ^= dwPolynomial;
        xbit >>= 1;
      }
    }
    return CRC32;
  };

  void lowStateCallback(const void* msg) {
    const unitree_hg::msg::dds_::LowState_* low_state =
        static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
    if (low_state->crc() !=
        crc32Core((uint32_t*)low_state,
                  (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
      std::cout << "low_state CRC Error" << std::endl;
      return;
    }
    memcpy(&low_state_, low_state, sizeof(unitree_hg::msg::dds_::LowState_));
    if (mode_machine_ != low_state->mode_machine()) {
      if (mode_machine_ == 0)
        std::cout << "G1 type: " << unsigned(low_state->mode_machine())
                  << std::endl;
      mode_machine_ = low_state->mode_machine();
    }
  }

  bool checkJointSafety(const G1DualArmLowCmd& cmd, const int& joint_index) {
    float desired_tau = cmd.getControlGainKp()[joint_index - kG1ArmIdxStart_] *
                            (cmd.getQ()[joint_index - kG1ArmIdxStart_] -
                             low_state_.motor_state()[joint_index].q()) +
                        cmd.getControlGainKd()[joint_index - kG1ArmIdxStart_] *
                            (cmd.getDq()[joint_index - kG1ArmIdxStart_] -
                             low_state_.motor_state()[joint_index].dq());
    if (desired_tau > kG1ArmTauLimit_[joint_index - kG1ArmIdxStart_]) {
      std::cout << "The command for arm joint(" << joint_index
                << ") will make joint torque exceed the limit, so the command "
                   "is ignored."
                << std::endl;
      return false;
    }
    return true;
  }

 private:
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
      lowcmd_publisher_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
      lowstate_subscriber_;
  unitree_hg::msg::dds_::LowState_ low_state_{};
  unitree_hg::msg::dds_::LowCmd_ low_cmd_{};
  uint8_t mode_machine_{0};
  uint8_t mode_{0};  // PR:0, AB:1

  const std::string kArmCmdTopic_{"rt/lowcmd"}, kStatesTopic_{"rt/lowstate"};
  const int kG1NumMotor_{29};
  const int kG1ArmIdxStart_{15};
  const unsigned int kG1ArmDof_{7};
  const std::array<float, 14> kG1ArmTauLimit_{25, 25, 25, 25, 25, 5, 5,
                                              25, 25, 25, 25, 25, 5, 5};
};

/**
 * @brief
 * @details Comm is the abbreviation for communication.
 */
class G1DualCommAPI : public DualArmAPI {
 public:
  G1DualCommAPI(const ros::NodeHandle& handle,
                const std::string& interface = "eth0")
      : handle_(handle) {
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    lowcmd_publisher_ =
        handle_.advertise<sensor_msgs::JointState>(kCmdTopic_, 1);
    lowstate_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            kStateTopic_));
    lowstate_subscriber_->InitChannel(
        std::bind(&G1DualCommAPI::lowStateCallback, this,
                  std::placeholders::_1),
        1);
    while (low_state_.tick() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // Initialize the command message
    msg_.name = kJointNames_;
    msg_.position.assign(kJointNames_.size(), 0);
    msg_.velocity.assign(kJointNames_.size(), 0);
    msg_.effort.assign(kJointNames_.size(), 0);
    for (int i{0}; i < 14; ++i) {
      msg_.position[2 * i] = low_state_.motor_state()[i + kG1ArmIdxStart_].q();
    }
  }
  void send() override { lowcmd_publisher_.publish(msg_); }
  void recv() override {}
  void setCmd(const G1DualArmLowCmd& cmd) override {
    msg_.header.stamp = ros::Time::now();
    msg_.name = kJointNames_;
    for (int i{0}; i < kG1ArmDof_; ++i) {
      msg_.position[2 * i] = cmd.left_arm.q[i];
      msg_.velocity[2 * i] = cmd.left_arm.dq[i];
      msg_.position[2 * i + 1] = cmd.left_arm.kp[i];
      msg_.velocity[2 * i + 1] = cmd.left_arm.kd[i];
      msg_.position[2 * (i + kG1ArmDof_)] = cmd.right_arm.q[i];
      msg_.velocity[2 * (i + kG1ArmDof_)] = cmd.right_arm.dq[i];
      msg_.position[2 * (i + kG1ArmDof_) + 1] = cmd.right_arm.kd[i];
      msg_.velocity[2 * (i + kG1ArmDof_) + 1] = cmd.right_arm.kd[i];
    }
  }
  void getState(G1DualArmLowState& state) override {
    for (int i{kG1ArmIdxStart_}; i < kG1ArmIdxStart_ + kG1ArmDof_; ++i) {
      state.left_arm.q[i - kG1ArmIdxStart_] = low_state_.motor_state()[i].q();
      state.left_arm.dq[i - kG1ArmIdxStart_] = low_state_.motor_state()[i].dq();
      state.left_arm.ddq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i].ddq();
      state.left_arm.tau[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i].tau_est();
      state.right_arm.q[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].q();
      state.right_arm.dq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].dq();
      state.right_arm.ddq[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].ddq();
      state.right_arm.tau[i - kG1ArmIdxStart_] =
          low_state_.motor_state()[i + kG1ArmDof_].tau_est();
    }
    state.tick = low_state_.tick();
  }

 protected:
  uint32_t crc32Core(const uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
      xbit = 1 << 31;
      data = ptr[i];
      for (uint32_t bits = 0; bits < 32; bits++) {
        if (CRC32 & 0x80000000) {
          CRC32 <<= 1;
          CRC32 ^= dwPolynomial;
        } else
          CRC32 <<= 1;
        if (data & xbit) CRC32 ^= dwPolynomial;
        xbit >>= 1;
      }
    }
    return CRC32;
  };

  void lowStateCallback(const void* msg) {
    const unitree_hg::msg::dds_::LowState_* low_state =
        static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
    if (low_state->crc() !=
        crc32Core((uint32_t*)low_state,
                  (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
      std::cout << "low_state CRC Error" << std::endl;
      return;
    }
    memcpy(&low_state_, low_state, sizeof(unitree_hg::msg::dds_::LowState_));
    if (mode_machine_ != low_state->mode_machine()) {
      if (mode_machine_ == 0)
        std::cout << "G1 type: " << unsigned(low_state->mode_machine())
                  << std::endl;
      mode_machine_ = low_state->mode_machine();
    }
  }

 protected:
  ros::NodeHandle handle_;
  uint8_t mode_machine_{0};
  unitree_hg::msg::dds_::LowState_ low_state_{};
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
      lowstate_subscriber_;
  ros::Publisher lowcmd_publisher_;

  const std::string kCmdTopic_{"/lqc/arm_cmd"};
  const std::string kStateTopic_{"rt/lowstate"};
  const std::vector<std::string> kJointNames_{
      "L_SHOULDER_PITCH", "L_SHOULDER_PITCH_GAIN",
      "L_SHOULDER_ROLL",  "L_SHOULDER_ROLL_GAIN",
      "L_SHOULDER_YAW",   "L_SHOULDER_YAW_GAIN",
      "L_ELBOW",          "L_ELBOW_GAIN",
      "L_WRIST_ROLL",     "L_WRIST_ROLL_GAIN",
      "L_WRIST_PITCH",    "L_WRIST_PITCH_GAIN",
      "L_WRIST_YAW",      "L_WRIST_YAW_GAIN",
      "R_SHOULDER_PITCH", "R_SHOULDER_PITCH_GAIN",
      "R_SHOULDER_ROLL",  "R_SHOULDER_ROLL_GAIN",
      "R_SHOULDER_YAW",   "R_SHOULDER_YAW_GAIN",
      "R_ELBOW",          "R_ELBOW_GAIN",
      "R_WRIST_ROLL",     "R_WRIST_ROLL_GAIN",
      "R_WRIST_PITCH",    "R_WRIST_PITCH_GAIN",
      "R_WRIST_YAW",      "R_WRIST_YAW_GAIN"};
  sensor_msgs::JointState msg_{};
  const int kG1NumMotor_{29};
  const int kG1ArmIdxStart_{15};
  const unsigned int kG1ArmDof_{7};
};

}  // namespace sdk
