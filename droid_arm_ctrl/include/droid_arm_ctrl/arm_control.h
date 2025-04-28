#pragma once

#include <mutex>
#include <thread>

// clang-format off
#include "droid_arm_ctrl/arm_model.h"
#include "droid_arm_ctrl/arm_api.h"
#include "droid_arm_ctrl/arm_planner.h"
#include "droid_arm_ctrl/utils.h"
#include "droid_arm_ctrl/logs.h"
// clang-format on
#include <actionlib/server/simple_action_server.h>

#include "droid_arm_ctrl/math_utils.h"
#include "dual_arm_as/ArmMotionAction.h"

namespace g1_controller {

enum class StateMachine { Init, Idle, Executing, UnSafe };
struct KeyFrame {
  double duration;
  Eigen::VectorXf q;
  Eigen::VectorXf dq;
  Eigen::VectorXf tau;
  double kp{80.}, kd{1.0};
};

using MotionProvider = std::function<void(std::deque<KeyFrame> &)>;

class MotionLibsRegistry {
 public:
  void registerMotion(const std::string &name, MotionProvider provider) {
    if (motion_libs_.find(name) != motion_libs_.end()) {
      ROS_WARN_STREAM(
          "Motion provider "
          << name << " already registered, Overwriting the motion provider");
    }
    // ROS_INFO_STREAM("Registering motion provider" << name);
    motion_libs_[name] = provider;
  }

  bool hasMotion(const std::string &name) {
    return motion_libs_.find(name) != motion_libs_.end();
  }

  MotionProvider &getMotion(const std::string &name) {
    if (motion_libs_.find(name) == motion_libs_.end()) {
      ROS_ERROR_STREAM("Motion provider "
                       << name << " not found, returning default action.");
      return motion_libs_["hello"];
    }
    return motion_libs_[name];
  }

  std::unordered_map<std::string, MotionProvider> &getAllMotion() {
    return motion_libs_;
  }

  void clear() { motion_libs_.clear(); }

  void listMotion() {
    for (const auto &motion : motion_libs_) {
      ROS_INFO_STREAM("Motion name: " << motion.first);
    }
  }

 private:
  std::unordered_map<std::string, MotionProvider> motion_libs_;
};

class G1ArmController {
 public:
  G1ArmController(const ros::NodeHandle &handle);
  ~G1ArmController();

  bool init();
  void start();
  void stop();
  void setArmApi(const std::string &name = "eth0") {
    api_ptr_ = std::make_unique<sdk::G1DualArmAPI>(name);
  }
  void setIntCommApi(const std::string &name = "eth0") {
    api_ptr_ = std::make_unique<sdk::G1DualCommAPI>(handle_, name);
  }
  void registerMotion();

  void communicationLoop() {
    utils::Rate rate(comm_freq_);
    while (running_) {
      {
        std::lock_guard<std::mutex> lock(mtx_);
        api_ptr_->setCmd(low_cmd_);
        api_ptr_->getState(low_state_);
      }
      api_ptr_->send();
      api_ptr_->recv();
      rate.sleep();
    }
  };

  void ctrlLoop() {
    utils::Rate rate(ctrl_freq_);
    while (running_) {
      if (!checkSafety(low_state_.getTau(),
                       1. / static_cast<double>(ctrl_freq_))) {
        ROS_WARN("Arm is not safe");
        arm_state_ = StateMachine::UnSafe;
      };
      StateMachine curr_state = arm_state_;
      if (curr_state == StateMachine::Executing && motion_server_->isActive()) {
        if (motion_server_->isPreemptRequested()) {
          preemptCallback();
          curr_state = arm_state_;
        }
      }
      ros::Time now = ros::Time::now();
      switch (curr_state) {
        case StateMachine::Init: {
          prev_key_frame_.q = low_state_.getQ();
          prev_key_frame_.dq = Eigen::VectorXf::Zero(14);
          prev_key_frame_.tau = Eigen::VectorXf::Zero(14);
          prev_key_frame_.duration = 2.0;
          prev_key_frame_.kp = 0.f;
          prev_key_frame_.kd = 1.f;
          segment_start_time_ = now;
          break;
        }
        case StateMachine::Idle: {
          segment_start_time_ = now;
          break;
        }
        case StateMachine::Executing: {
          if (motion_seq_.empty()) {
            arm_state_ = StateMachine::Idle;
            if (motion_server_->isActive()) {
              motion_result_.success = false;
              motion_server_->setAborted(
                  motion_result_, "Internal Error: Empty motion sequence");
            }
            break;
          }
          const KeyFrame &target_kf = motion_seq_.front();
          double time_in_segment = (now - segment_start_time_).toSec();
          if (time_in_segment >= target_kf.duration) {
            // End the segment
            prev_key_frame_ = target_kf;
            motion_seq_.pop_front();
            if (motion_seq_.empty()) {
              // End the motion
              arm_state_ = StateMachine::Idle;
              if (motion_server_->isActive()) {
                motion_result_.success = true;
                motion_server_->setSucceeded(motion_result_);
              }
            } else {
              // Start next segment motion
              segment_start_time_ = now;
            }
            {
              std::lock_guard<std::mutex> lock(mtx_);
              low_cmd_.setQ(prev_key_frame_.q);
              low_cmd_.setControlGain(prev_key_frame_.kp, prev_key_frame_.kd);
              low_cmd_.setDq(prev_key_frame_.dq);
              low_cmd_.setTau(prev_key_frame_.tau);
            }
          } else {
            double ratio = time_in_segment / target_kf.duration;
            sdk::G1DualArmLowCmd cmd =
                interpolateState(prev_key_frame_, target_kf, ratio);
            {
              std::lock_guard<std::mutex> lock(mtx_);
              low_cmd_ = cmd;
            }
          }
          break;
        }
        case StateMachine::UnSafe: {
          prev_key_frame_.q = low_state_.getQ();
          prev_key_frame_.dq = Eigen::VectorXf::Zero(14);
          prev_key_frame_.tau = Eigen::VectorXf::Zero(14);
          prev_key_frame_.duration = 0.0;
          prev_key_frame_.kp = 0.f;
          prev_key_frame_.kd = 1.f;
          segment_start_time_ = now;
          {
            std::lock_guard<std::mutex> lock(mtx_);
            low_cmd_.setQ(low_state_.getQ());
            low_cmd_.setControlGain(0.f, 1.f);
            low_cmd_.setDq(Eigen::VectorXf::Zero(14));
            low_cmd_.setTau(Eigen::VectorXf::Zero(14));
          }
          break;
        }
        default:
          break;
      }
      logger_.publishCmd(low_cmd_);
      logger_.publishState(low_state_);
      rate.sleep();
    }
  }

  void ctrlStep();

  void goalCallback();

  void preemptCallback();

  sdk::G1DualArmLowCmd interpolateState(const KeyFrame &start_kf,
                                        const KeyFrame &end_kf,
                                        const double &ratio);

  void planInitMotion();
  void planHelloMotion(std::deque<KeyFrame> &motion_seq);
  void planSelfIntroductionMotion(std::deque<KeyFrame> &motion_seq);
  void planShrugMotion(std::deque<KeyFrame> &motion_seq);
  void planPointMotion(std::deque<KeyFrame> &motion_seq);
  void planPromotionMotion(std::deque<KeyFrame> &motion_seq);
  void planAnswer1Motion(std::deque<KeyFrame> &motion_seq);
  void planAnswer2Motion(std::deque<KeyFrame> &motion_seq);
  void planPickupMotion(std::deque<KeyFrame> &motion_seq);
  void planPlaceMotion(std::deque<KeyFrame> &motion_seq);

  bool checkSafety(const Eigen::Ref<const Eigen::VectorXf> &tau,
                   const double &dt) {
    int n = ((tau.cwiseAbs() - joint_tau_limit_).array() > 0.).count();
    if (n > 0) {
      max_tau_time_ += dt;
    } else {
      max_tau_time_ = std::max(0., max_tau_time_ - dt);
    }
    bool safety = (max_tau_time_ <= kTauTimeLimit_);
    if (!safety) arm_state_ = StateMachine::UnSafe;
    return safety;
  }

 protected:
  ros::NodeHandle handle_;
  sdk::G1DualArmLowCmd low_cmd_{};
  sdk::G1DualArmLowState low_state_{};
  std::thread communication_thread_, ctrl_thread_;
  std::atomic<bool> running_{false};
  std::unique_ptr<sdk::DualArmAPI> api_ptr_;
  std::unique_ptr<g1_dual_arm::G1DualArmPlanner> arm_planner_;
  std::mutex mtx_;
  int ctrl_freq_{200}, comm_freq_{200};

  StateMachine arm_state_{StateMachine::Init};
  KeyFrame prev_key_frame_;
  ros::Time segment_start_time_;
  using ArmMotionServer =
      actionlib::SimpleActionServer<dual_arm_as::ArmMotionAction>;
  std::unique_ptr<ArmMotionServer> motion_server_;
  dual_arm_as::ArmMotionResult motion_result_;
  dual_arm_as::ArmMotionFeedback motion_feedback_;
  math_utils::QuinticInterpolationFn<Eigen::VectorXf> q_interp_fn_;
  math_utils::CubicInterpolationFn<Eigen::VectorXf> c_interp_fn_;
  std::deque<KeyFrame> motion_seq_;
  MotionLibsRegistry motion_libs_;

  Loggers logger_;

  Eigen::Matrix<float, 14, 1> joint_tau_limit_;
  Eigen::Matrix<float, 14, 1> joint_home_;
  const double kTauTimeLimit_{10.0};
  double max_tau_time_{0.0};
};
}  // namespace g1_controller
