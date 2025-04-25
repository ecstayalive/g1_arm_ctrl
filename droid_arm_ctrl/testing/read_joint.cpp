#include <chrono>
#include <cmath>
#include <thread>

#include "droid_arm_ctrl/arm_api.h"
#include "droid_arm_ctrl/utils.h"

class TestApi {
 public:
  TestApi() {
    api_ptr_ = std::make_unique<sdk::G1DualArmAPI>();
    if (communication_.joinable()) {
      communication_.join();
    }
    communication_ = std::thread([this]() {
      utils::Rate rate(200);
      while (communication_enabled_) {
        api_ptr_->setCmd(low_cmd_);
        api_ptr_->send();
        api_ptr_->recv();
        api_ptr_->getState(low_state_);
        rate.sleep();
      }
    });
    while (low_state_.tick == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    low_cmd_.setControlGain(0.f, 1.f);
    low_cmd_.setQ(low_state_.getQ());
  }

  ~TestApi() {
    communication_enabled_ = false;
    if (communication_.joinable()) {
      communication_.join();
    }
  }

 public:
  std::unique_ptr<sdk::G1DualArmAPI> api_ptr_;
  std::thread communication_;
  bool communication_enabled_{true};

  sdk::G1DualArmLowCmd low_cmd_;
  sdk::G1DualArmLowState low_state_;
};

int main(int argc, char **argv) {
  TestApi test_api;

  float control_dt{0.02f};
  float max_joint_vel = 0.5f;
  float max_dq = max_joint_vel * control_dt;
  std::chrono::milliseconds sleep_time =
      std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));
  Eigen::VectorXf init_pos(Eigen::VectorXf::Zero(14));
  Eigen::VectorXf target_pos(Eigen::VectorXf::Zero(14));
  // clang-format off
  target_pos << 0.f, M_PI_2f32, 0.f, M_PI_2f32, 0.f, 0.f, 0.f,
                0.f, -M_PI_2f32, 0.f, M_PI_2f32, 0.f, 0.f, 0.f;
  // clang-format on

  // Initialize arm
  Eigen::VectorXf cur_q(14), cmd_q(init_pos);
  test_api.low_state_.getQ(cur_q);
  float period = 2.f;
  int num_time_steps = static_cast<int>(period / control_dt);
  for (int i{0}; i < num_time_steps; ++i) {
    float phase = static_cast<float>(i) / num_time_steps;
    test_api.low_cmd_.setQ(cur_q * (1 - phase) + phase * init_pos);
    test_api.low_cmd_.setDq(Eigen::VectorXf::Zero(14));
    test_api.low_cmd_.setTau(Eigen::VectorXf::Zero(14));
    test_api.low_cmd_.setControlGain(40.f, 1.0f);
    std::this_thread::sleep_for(sleep_time);
  }

  // Lift arm
  period = 5.0f;
  num_time_steps = static_cast<int>(period / control_dt);
  for (int i{0}; i < num_time_steps; ++i) {
    cmd_q += (target_pos - cmd_q).cwiseMax(-max_dq).cwiseMin(max_dq);
    test_api.low_cmd_.setQ(cmd_q);
    test_api.low_cmd_.setDq(Eigen::VectorXf::Zero(14));
    test_api.low_cmd_.setTau(Eigen::VectorXf::Zero(14));
    test_api.low_cmd_.setControlGain(40.f, 1.0f);
    std::this_thread::sleep_for(sleep_time);
  }

  // Reset arm
  period = 5.0f;
  num_time_steps = static_cast<int>(period / control_dt);
  for (int i{0}; i < num_time_steps; ++i) {
    cmd_q += (init_pos - cmd_q).cwiseMax(-max_dq).cwiseMin(max_dq);
    test_api.low_cmd_.setQ(cmd_q);
    test_api.low_cmd_.setDq(Eigen::VectorXf::Zero(14));
    test_api.low_cmd_.setTau(Eigen::VectorXf::Zero(14));
    test_api.low_cmd_.setControlGain(40.f, 1.f);
    std::this_thread::sleep_for(sleep_time);
  }

  return 0;
}
