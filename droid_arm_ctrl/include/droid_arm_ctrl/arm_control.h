#pragma once

#include <mutex>
#include <thread>

// clang-format off
#include "droid_arm_ctrl/arm_model.h"
#include "droid_arm_ctrl/arm_api.h"
#include "droid_arm_ctrl/arm_planner.h"
#include "droid_arm_ctrl/utils.h"
// clang-format on
#include <actionlib/server/simple_action_server.h>

#include "droid_arm_ctrl/math_utils.h"
#include "dual_arm_as/ArmMotionAction.h"

namespace g1_controller {

enum class MotionLibs {
  Hello,
  SelfIntroduction,
  Shrug,
  Promotion,
  Point,
  Answer1,
  Answer2,
  Pickup,
  Place
};
enum class StateMachine { Init, Idle, Executing, UnSafe };
const std::unordered_map<std::string, MotionLibs> kMotionLibMap{
    {"hello", MotionLibs::Hello},
    {"self_introduction", MotionLibs::SelfIntroduction},
    {"shrug", MotionLibs::Shrug},
    {"point", MotionLibs::Point},
    {"promotion", MotionLibs::Promotion},
    {"answer1", MotionLibs::Answer1},
    {"answer2", MotionLibs::Answer2},
    {"pickup", MotionLibs::Pickup},
    {"place", MotionLibs::Place}};

struct KeyFrame {
  double duration;
  Eigen::VectorXf q;
  Eigen::VectorXf dq;
  Eigen::VectorXf tau;
  double kp{40.}, kd{1.0};
};

using MotionProvider = std::function<void(std::deque<KeyFrame> &)>;

class MotionLibsRegistry {
 public:
  void registerMotionProvider(const std::string &name,
                              MotionProvider provider) {
    if (motion_provider_.find(name) != motion_provider_.end()) {
      ROS_WARN_STREAM(
          "Motion provider "
          << name << " already registered, Overwriting the motion provider");
    }
    motion_provider_[name] = provider;
  }

 private:
  std::unordered_map<std::string, MotionProvider> motion_provider_;
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
  void planHelloMotion();
  void planSelfIntroductionMotion();
  void planShrugMotion();
  void planPointMotion();
  void planPromotionMotion();
  void planAnswer1Motion();
  void planAnswer2Motion();
  void planPickupMotion();
  void planPlaceMotion();

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
  sdk::G1DualArmLowCmd low_cmd_;
  sdk::G1DualArmLowState low_state_;
  std::thread communication_thread_, ctrl_thread_;
  std::atomic<bool> running_{false};
  std::unique_ptr<sdk::DualArmAPI> api_ptr_;
  std::unique_ptr<g1_dual_arm::G1DualArmPlanner> arm_planner_;
  std::mutex mtx_;
  ros::Time segment_start_time_;
  std::deque<KeyFrame> motion_seq_;
  KeyFrame prev_key_frame_;
  StateMachine arm_state_{StateMachine::Init};
  int ctrl_freq_{200}, comm_freq_{200};

  using ArmMotionServer =
      actionlib::SimpleActionServer<dual_arm_as::ArmMotionAction>;
  std::unique_ptr<ArmMotionServer> motion_server_;
  dual_arm_as::ArmMotionResult motion_result_;
  dual_arm_as::ArmMotionFeedback motion_feedback_;
  math_utils::QuinticInterpolationFn<Eigen::VectorXf> q_interp_fn_;
  math_utils::CubicInterpolationFn<Eigen::VectorXf> c_interp_fn_;

  Eigen::Matrix<float, 14, 1> joint_tau_limit_;
  const double kTauTimeLimit_{2};
  double max_tau_time_{0.0};
};
}  // namespace g1_controller
