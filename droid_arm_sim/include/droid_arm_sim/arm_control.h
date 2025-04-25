#pragma once
#include "droid_arm_ctrl/arm_control.h"
#include "gazebo_arm_api.h"

namespace g1_controller {
class G1ArmSimController : public G1ArmController {
 public:
  G1ArmSimController(const ros::NodeHandle& handle) : G1ArmController(handle) {}

  bool init() {
    arm_planner_ = std::make_unique<g1_dual_arm::G1DualArmPlanner>(handle_);
    setSimArmApi();
    if (!api_ptr_) {
      ROS_ERROR("Arm API is not set");
      return false;
    }
    motion_server_ = std::make_unique<ArmMotionServer>(
        handle_, "/droid_arm_ctrl/arm_motion", false);
    motion_server_->registerGoalCallback(
        std::bind(&G1ArmSimController::goalCallback, this));
    registerMotion();
    return true;
  }

  void setSimArmApi() {
    api_ptr_ = std::make_unique<sdk::G1DualArmGazeboAPI>(handle_);
  }
};
}  // namespace g1_controller
