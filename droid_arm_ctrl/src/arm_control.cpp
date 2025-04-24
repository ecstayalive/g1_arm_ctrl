#include "droid_arm_ctrl/arm_control.h"

#include "droid_arm_ctrl/math_utils.h"

namespace g1_controller {

G1Controller::G1Controller(const ros::NodeHandle &handle)
    : handle_(handle), arm_state_{StateMachine::Init} {
  //   arm_model_ = std::make_unique<g1_dual_arm::G1DualArmModel>(handle);
  // clang-format off
  joint_tau_limit_ << 25, 25, 25, 25, 25, 5.0, 5.0,
                      25, 25, 25, 25, 25, 5.0, 5.0;
  // clang-format on
  init();
  start();
}

G1Controller::~G1Controller() { stop(); }

bool G1Controller::init() {
  arm_planner_ = std::make_unique<g1_dual_arm::G1DualArmPlanner>(handle_);
  bool use_sim;
  handle_.param("use_sim", use_sim, false);
  if (use_sim) {
    setSimArmApi();
  } else {
    setIntCommApi();
  }
  if (!api_ptr_) {
    ROS_ERROR("Arm API is not set");
    return false;
  }
  motion_server_ = std::make_unique<ArmMotionServer>(
      handle_, "/droid_arm_ctrl/arm_motion", false);
  motion_server_->registerGoalCallback(
      std::bind(&G1Controller::goalCallback, this));
  return true;
}

void G1Controller::start() {
  if (running_) {
    ROS_WARN("Controller is already running");
    return;
  }
  if (!api_ptr_) {
    ROS_ERROR("Cannot start controller: API not initialized");
    return;
  }
  if (!arm_planner_) {
    ROS_ERROR("Cannot start controller: Planner not initialized");
    return;
  }
  running_ = true;
  communication_thread_ = std::thread(&G1Controller::communicationLoop, this);
  while (low_state_.tick == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  prev_key_frame_.q = low_state_.getQ();
  prev_key_frame_.dq = low_state_.getDq();
  prev_key_frame_.tau = low_state_.getTau();
  prev_key_frame_.kp = 0.f;
  prev_key_frame_.kd = 0.f;
  low_cmd_.setControlGain(0.f, 0.f);
  low_cmd_.setQ(low_state_.getQ());
  ctrl_thread_ = std::thread(&G1Controller::ctrlLoop, this);
  motion_server_->start();
  ROS_INFO("Controller started");
}

void G1Controller::stop() {
  running_ = false;
  if (motion_server_) {
    motion_server_->shutdown();
  }
  if (ctrl_thread_.joinable()) {
    ctrl_thread_.join();
  }
  if (communication_thread_.joinable()) {
    ctrl_thread_.join();
  }
  api_ptr_.reset();
  arm_planner_.reset();
  motion_server_.reset();
  arm_state_ = StateMachine::Init;
}

void G1Controller::ctrlStep() {}

void G1Controller::goalCallback() {
  if (arm_state_ != StateMachine::Idle && arm_state_ != StateMachine::Init) {
    ROS_WARN("Arm is busy, cannot accept new goal");
    motion_result_.success = false;
    motion_server_->setAborted(motion_result_, "Arm is busy");
    return;
  }
  dual_arm_as::ArmMotionGoalConstPtr goal = motion_server_->acceptNewGoal();
  if (kMotionLibMap.find(goal->motion) == kMotionLibMap.end()) {
    ROS_WARN("Invalid motion name: %s", goal->motion.c_str());
    motion_result_.success = false;
    motion_server_->setAborted(motion_result_, "Invalid motion name");
    return;
  }
  switch (kMotionLibMap.at(goal->motion)) {
    case MotionLibs::Hello:
      planHelloMotion();
      break;
    case MotionLibs::SelfIntroduction:
      planSelfIntroductionMotion();
      break;
    case MotionLibs::Shrug:
      planShrugMotion();
      break;
    case MotionLibs::Point:
      planPointMotion();
      break;
    case MotionLibs::Promotion:
      planPromotionMotion();
      break;
    case MotionLibs::Answer1:
      planAnswer1Motion();
      break;
    case MotionLibs::Answer2:
      planAnswer2Motion();
      break;
    case MotionLibs::Pickup:
      planPickupMotion();
      break;
    case MotionLibs::Place:
      planPlaceMotion();
      break;
    default:
      break;
  }
  arm_state_ = StateMachine::Executing;
}

void G1Controller::preemptCallback() {}

sdk::G1DualArmLowCmd G1Controller::interpolateState(const KeyFrame &start_kf,
                                                    const KeyFrame &end_kf,
                                                    const double &ratio) {
  sdk::G1DualArmLowCmd cmd;
  q_interp_fn_.setPolyInterpKernel(1.0, start_kf.q, end_kf.q);
  cmd.setQ(q_interp_fn_.solve(ratio));
  // q_interp_fn_.setPolyInterpolationKernel(1.0, start_kf.dq, end_kf.dq);
  // cmd.setDq(q_interp_fn_.solve(ratio));
  // q_interp_fn_.setPolyInterpolationKernel(1.0, start_kf.tau, end_kf.tau);
  // cmd.setTau(q_interp_fn_.solve(ratio));
  cmd.setControlGain(end_kf.kp, end_kf.kd);
  return cmd;
}

void G1Controller::planPickupMotion() {
  KeyFrame kf;
  Eigen::Isometry3d left_arm_target_pose = Eigen::Isometry3d::Identity(),
                    right_arm_target_pose = Eigen::Isometry3d::Identity();
  Eigen::VectorXf goal_q(14), prev_q(14);
  bool left_find_ik, right_find_ik;
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  left_arm_target_pose.translation() << 0.05, 0.3, 0.8;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_planner_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                                left_arm_target_pose,
                                                low_state_.getQ(), goal_q);
  right_find_ik = arm_planner_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose,
      low_state_.getQ(), goal_q);
  kf.q = goal_q;
  kf.dq = Eigen::VectorXf::Zero(14);
  kf.tau = Eigen::VectorXf::Zero(14);
  kf.kp = 40.f;
  kf.kd = 1.f;
  kf.duration = 2.0;
  motion_seq_.push_back(kf);
  // stage 2
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitZ()));
  prev_q = goal_q;
  left_arm_target_pose.translation() << 0.27, 0.15, 0.69;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_planner_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                                left_arm_target_pose,
                                                low_state_.getQ(), goal_q);
  right_find_ik = arm_planner_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose,
      low_state_.getQ(), goal_q);
  kf.q = goal_q;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
  // stage 3
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitY()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitY()));
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitZ()));
  prev_q = goal_q;
  left_arm_target_pose.translation() << 0.27, 0.15, 0.9;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_planner_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                                left_arm_target_pose,
                                                low_state_.getQ(), goal_q);
  right_find_ik = arm_planner_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose,
      low_state_.getQ(), goal_q);
  kf.q = goal_q;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
}

void G1Controller::planPlaceMotion() {
  KeyFrame kf;
  Eigen::Isometry3d left_arm_target_pose = Eigen::Isometry3d::Identity(),
                    right_arm_target_pose = Eigen::Isometry3d::Identity();
  Eigen::VectorXf goal_q(14), prev_q(14);
  bool left_find_ik, right_find_ik;
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  left_arm_target_pose.translation() << 0.27, 0.15, 0.69;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_planner_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                                left_arm_target_pose,
                                                low_state_.getQ(), goal_q);
  right_find_ik = arm_planner_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose,
      low_state_.getQ(), goal_q);
  kf.q = goal_q;
  kf.dq = Eigen::VectorXf::Zero(14);
  kf.tau = Eigen::VectorXf::Zero(14);
  kf.kp = 40.f;
  kf.kd = 1.f;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
  // stage 2
  left_arm_target_pose.setIdentity();
  right_arm_target_pose.setIdentity();
  left_arm_target_pose.rotate(
      Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitZ()));
  right_arm_target_pose.rotate(
      Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitZ()));
  prev_q = goal_q;
  left_arm_target_pose.translation() << 0.20, 0.24, 0.69;
  right_arm_target_pose.translation() = left_arm_target_pose.translation();
  right_arm_target_pose.translation().y() =
      -right_arm_target_pose.translation().y();
  left_find_ik = arm_planner_->inverseKinematic(g1_dual_arm::G1ArmEnum::LeftArm,
                                                left_arm_target_pose,
                                                low_state_.getQ(), goal_q);
  right_find_ik = arm_planner_->inverseKinematic(
      g1_dual_arm::G1ArmEnum::RightArm, right_arm_target_pose,
      low_state_.getQ(), goal_q);
  kf.q = goal_q;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
}

void G1Controller::planHelloMotion() {
  KeyFrame kf = prev_key_frame_;
  Eigen::VectorXf q1(7), q2(7);
  q1 << -0.7, -0.2, 0.32, -0.469207, -1.58333, -0.0643305, -0.488429;
  q2 << -0.7, -0.2, -0.41319, -0.442494, -1., 0.00611996, -0.45914;
  //
  kf.q.tail<7>() << -0.7, -0.2, -0.21748, -0.469207, -1.58333, -0.0643305,
      -0.488429;
  kf.duration = 2.5;
  kf.kp = 40.f;
  kf.kd = 1.f;
  motion_seq_.push_back(kf);
  //
  for (int i{0}; i < 5; ++i) {
    kf.q.tail<7>() = q1;
    kf.duration = 1.0;
    motion_seq_.push_back(kf);
    kf.q.tail<7>() = q2;
    kf.duration = 1.0;
    motion_seq_.push_back(kf);
  }
}

void G1Controller::planSelfIntroductionMotion() {
  KeyFrame kf = prev_key_frame_;
  kf.q.tail<7>() << -0.75, -0.7, 0.8, -0.75, 1.2, 0, 0;
  kf.dq = Eigen::VectorXf::Zero(14);
  kf.tau = Eigen::VectorXf::Zero(14);
  kf.kp = 40.f;
  kf.kd = 1.f;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
  kf.duration = 4.0;
  motion_seq_.push_back(kf);
  kf.q.tail<7>() << 0, -0.62, 0, 1.04, 0, 0, 0;
  kf.duration = 3.5;
  motion_seq_.push_back(kf);
}

void G1Controller::planShrugMotion() {
  KeyFrame kf = prev_key_frame_;
  // clang-format off
  kf.q << -0.2, 0.2, 0.2, 0.6, -1.5, 0, 0,
          -0.2, -0.2, -0.2, 0.6, 1.5, 0, 0;
  // clang-format on
  kf.dq = Eigen::VectorXf::Zero(14);
  kf.tau = Eigen::VectorXf::Zero(14);
  kf.kp = 40.f;
  kf.kd = 1.f;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
  // kf.duration = 4.0;
  // motion_seq_.push_back(kf);
  // kf.q.tail<7>() << 0, -0.62, 0, 1.04, 0, 0, 0;
  // kf.duration = 3.5;
  // motion_seq_.push_back(kf);
}

void G1Controller::planPromotionMotion() {
  KeyFrame kf = prev_key_frame_;
  kf.q.tail<7>() << 0, -0.9, -1.5, 0.2, 1.2, 0, 0.1;
  kf.duration = 3.0;
  kf.kp = 40.f;
  kf.kd = 1.f;
  motion_seq_.push_back(kf);
  kf.duration = 2.0;
  motion_seq_.push_back(kf);
  kf.q.tail<7>() << 0, -0.62, 0, 1.04, 0, 0, 0;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
}

void G1Controller::planPointMotion() {
  KeyFrame kf = prev_key_frame_;
  // Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  // Eigen::VectorXf goal_q(14);
  // bool left_find_ik, find_ik;
  // target_pose.rotate(Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitY()));
  // target_pose.translation() << 0.3, -0.15, 0.95;
  // find_ik = arm_planner_->inverseKinematic(
  //     g1_dual_arm::G1ArmEnum::RightArm, target_pose, low_state_.getQ(),
  //     goal_q);
  // std::cout << "find_ik: " << find_ik << "\ngoal_q: " << goal_q.transpose()
  //           << std::endl;
  kf.q.tail<7>() << -0.840968, -0.0155465, -0.264735, -0.191843, -0.0759665,
      0.207531, 0.0714225;
  kf.dq = Eigen::VectorXf::Zero(14);
  kf.tau = Eigen::VectorXf::Zero(14);
  kf.kp = 40.f;
  kf.kd = 1.f;
  kf.duration = 2.5;
  motion_seq_.push_back(kf);
  // target_pose.rotate(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
  // target_pose.translation() << 0.4, -0.15, 0.95;
  // find_ik = arm_planner_->inverseKinematic(
  //     g1_dual_arm::G1ArmEnum::RightArm, target_pose, low_state_.getQ(),
  //     goal_q);
  // std::cout << "find_ik: " << find_ik << "\ngoal_q: " << goal_q.transpose()
  //           << std::endl;
  kf.q.tail<7>() << -1.29092, -0.170733, -0.335521, 0.697948, -0.0757133,
      0.310431, 0.0347134;
  kf.duration = 0.8;
  motion_seq_.push_back(kf);
  kf.q.tail<7>() << -0.840968, -0.0155465, -0.264735, -0.191843, -0.0759665,
      0.207531, 0.0714225;
  kf.duration = 0.8;
  motion_seq_.push_back(kf);
  kf.q.tail<7>() << 0, -0.62, 0, 1.04, 0, 0, 0;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
}

void G1Controller::planAnswer1Motion() {
  KeyFrame kf = prev_key_frame_;
  // Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  // Eigen::VectorXf goal_q(14);
  // bool left_find_ik, find_ik;
  // target_pose.rotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));
  // target_pose.translation() << 0.22, -0.23, 1.0;
  // find_ik = arm_planner_->inverseKinematic(
  //     g1_dual_arm::G1ArmEnum::RightArm, target_pose, low_state_.getQ(),
  //     goal_q);
  // std::cout << "find_ik: " << find_ik << "\ngoal_q: " << goal_q.transpose()
  //           << std::endl;
  kf.q.tail<7>() << -0.93152, -0.3, -0.2, -0.256451, 0.279513, -0.329058, 0.1;
  kf.duration = 3.0;
  kf.kp = 40.f;
  kf.kd = 1.f;
  motion_seq_.push_back(kf);
  kf.duration = 2.0;
  motion_seq_.push_back(kf);
  kf.q.tail<7>() << 0, -0.62, 0, 1.04, 0, 0, 0;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
}

void G1Controller::planAnswer2Motion() {
  KeyFrame kf = prev_key_frame_;
  // Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  // Eigen::VectorXf goal_q(14);
  // bool left_find_ik, find_ik;
  // target_pose.rotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()));
  // target_pose.translation() << 0.22, -0.23, 1.0;
  // find_ik = arm_planner_->inverseKinematic(
  //     g1_dual_arm::G1ArmEnum::RightArm, target_pose, low_state_.getQ(),
  //     goal_q);
  // std::cout << "find_ik: " << find_ik << "\ngoal_q: " << goal_q.transpose()
  //           << std::endl;
  kf.q.tail<7>() << -1., -0.6, -1, 0.6, 1.2, 0, 0.1;
  kf.duration = 3.0;
  kf.kp = 40.f;
  kf.kd = 1.f;
  motion_seq_.push_back(kf);
  kf.duration = 2.0;
  motion_seq_.push_back(kf);
  kf.q.tail<7>() << 0, -0.62, 0, 1.04, 0, 0, 0;
  kf.duration = 3.0;
  motion_seq_.push_back(kf);
}

}  // namespace g1_controller
