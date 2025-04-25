#pragma once
// clang-format off
#include "arm_model.h"
// clang-format on
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <pluginlib/class_loader.h>

namespace g1_dual_arm {
class G1DualArmPlanner {
 public:
  explicit G1DualArmPlanner(const ros::NodeHandle &handle) : handle_(handle) {
    robot_model_loader::RobotModelLoader robot_model_loader(
        "robot_description");
    robot_model_ = robot_model_loader.getModel();
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();
    left_arm_model_group_ = robot_model_->getJointModelGroup("left_arm");
    right_arm_model_group_ = robot_model_->getJointModelGroup("right_arm");
    planning_scene_ =
        std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_scene_->getCurrentStateNonConst().setToDefaultValues(
        left_arm_model_group_, "left_home");
    planning_scene_->getCurrentStateNonConst().setToDefaultValues(
        right_arm_model_group_, "right_home");
    // Trajectory planning algorithm
    std::string planner_plugin_name;
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>>
        planner_plugin_loader_;
    if (!handle_.getParam("/ompl/planning_plugin", planner_plugin_name))
      ROS_FATAL_STREAM("Could not find planner plugin name: " +
                       planner_plugin_name);
    try {
      planner_plugin_loader_.reset(
          new pluginlib::ClassLoader<planning_interface::PlannerManager>(
              "moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException &ex) {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader "
                       << ex.what());
    }
    planner_manager_.reset(
        planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name));
    if (!planner_manager_->initialize(robot_model_, handle_.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '"
                    << planner_manager_->getDescription() << "'");
    // Get joint limit of the robot model
    std::vector<std::string> joint_names =
        robot_model_->getActiveJointModelNames();
    for (int i{0}; i < joint_names.size(); ++i) {
      std::string joint_name{joint_names[i]};
      const moveit::core::JointModel *joint_model =
          robot_model_->getJointModel(joint_name);
      const moveit::core::VariableBounds &bounds =
          joint_model->getVariableBounds(joint_model->getVariableNames()[0]);
      joint_limit_.row(i) << bounds.min_position_, bounds.max_position_;
    }
    // std::cout << "joint limit:\n" << joint_limit_ << std::endl;
  }

  void updateRobotState(const Eigen::Ref<const Eigen::VectorXf> &q,
                        const Eigen::Ref<const Eigen::VectorXf> &dq,
                        const Eigen::Ref<const Eigen::VectorXf> &ddq) {
    robot_state_->setJointGroupPositions(left_arm_model_group_,
                                         q.head<7>().cast<double>());
    robot_state_->setJointGroupPositions(right_arm_model_group_,
                                         q.tail<7>().cast<double>());
    robot_state_->setJointGroupVelocities(left_arm_model_group_,
                                          dq.head<7>().cast<double>());
    robot_state_->setJointGroupVelocities(right_arm_model_group_,
                                          dq.tail<7>().cast<double>());
    robot_state_->setJointGroupAccelerations(left_arm_model_group_,
                                             ddq.head<7>().cast<double>());
    robot_state_->setJointGroupAccelerations(right_arm_model_group_,
                                             ddq.tail<7>().cast<double>());
    robot_state_->update();
  }

  bool inverseKinematic(G1ArmEnum arm_id, const Eigen::Isometry3d &target_pose,
                        const Eigen::Ref<const Eigen::VectorXf> &guess_q,
                        Eigen::Ref<Eigen::VectorXf> q,
                        const double &timeout = 0.005) {
    bool find_ik{false};
    std::vector<double> q_values;
    switch (arm_id) {
      case G1ArmEnum::LeftArm: {
        find_ik = robot_state_->setFromIK(left_arm_model_group_, target_pose,
                                          timeout);
        robot_state_->copyJointGroupPositions(left_arm_model_group_, q_values);
        if (find_ik) {
          q.head<7>() =
              Eigen::Map<Eigen::VectorXd>(q_values.data(), q_values.size())
                  .cast<float>();
        } else {
          q.head<7>().setZero();
        }
        break;
      }
      case G1ArmEnum::RightArm: {
        find_ik = robot_state_->setFromIK(right_arm_model_group_, target_pose,
                                          timeout);
        robot_state_->copyJointGroupPositions(right_arm_model_group_, q_values);
        if (find_ik) {
          q.tail<7>() =
              Eigen::Map<Eigen::VectorXd>(q_values.data(), q_values.size())
                  .cast<float>();
        } else {
          q.tail<7>().setZero();
        }
        break;
      }
    }
    // robot_state_->setToDefaultValues();
    return find_ik;
  }

  Eigen::MatrixXf getJacobian(G1ArmEnum arm_id) {
    Eigen::MatrixXd jacobian;
    switch (arm_id) {
      case G1ArmEnum::LeftArm: {
        jacobian = robot_state_->getJacobian(left_arm_model_group_,
                                             Eigen::Vector3d::Zero());
      }
      case G1ArmEnum::RightArm: {
        jacobian = robot_state_->getJacobian(right_arm_model_group_,
                                             Eigen::Vector3d::Zero());
      }
    }
    return jacobian.cast<float>();
  }

  bool constrainJointPos(Eigen::Ref<Eigen::VectorXf> joint_q) {
    Eigen::VectorXf low_distance = joint_q - joint_limit_.col(0).cast<float>();
    Eigen::VectorXf high_distance = joint_limit_.col(1).cast<float>() - joint_q;
    joint_q = joint_q.cwiseMax(joint_limit_.col(0).cast<float>())
                  .cwiseMin(joint_limit_.col(1).cast<float>());
    auto out_of_limit = low_distance.cwiseMin(high_distance).array() < 0.f;
    // std::cout << "joint distance : " << out_of_limit.transpose() <<
    // std::endl;
    return out_of_limit.count() ? true : false;
  }

  void planning(const Eigen::Ref<const Eigen::VectorXf> &left_arm_target_q,
                const Eigen::Ref<const Eigen::VectorXf> &right_arm_target_q) {
    const std::vector<double> left_arm_target_joint_pos(
        left_arm_target_q.data(),
        left_arm_target_q.data() + left_arm_target_q.size()),
        right_arm_target_joint_pos(
            right_arm_target_q.data(),
            right_arm_target_q.data() + right_arm_target_q.size());
    moveit::core::RobotState goal_state = planning_scene_->getCurrentState();
    goal_state.setJointGroupPositions(left_arm_model_group_,
                                      left_arm_target_joint_pos);
    goal_state.setJointGroupPositions(right_arm_model_group_,
                                      right_arm_target_joint_pos);
    planning_interface::MotionPlanRequest left_arm_req, right_arm_req;
    planning_interface::MotionPlanResponse left_arm_res, right_arm_res;
    moveit_msgs::Constraints left_arm_goal, right_arm_goal;
    planning_interface::PlanningContextPtr left_context, right_context;
    left_arm_req.group_name = "g1_left_arm";
    right_arm_req.group_name = "g1_right_arm";
    left_arm_goal = kinematic_constraints::constructGoalConstraints(
        goal_state, left_arm_model_group_);
    right_arm_goal = kinematic_constraints::constructGoalConstraints(
        goal_state, right_arm_model_group_);
    left_arm_req.goal_constraints.push_back(left_arm_goal);
    right_arm_req.goal_constraints.push_back(right_arm_goal);
    left_context = planner_manager_->getPlanningContext(
        planning_scene_, left_arm_req, left_arm_res.error_code_);
    right_context = planner_manager_->getPlanningContext(
        planning_scene_, right_arm_req, right_arm_res.error_code_);
    left_context->solve(left_arm_res);
    right_context->solve(right_arm_res);
    if (left_arm_res.error_code_.val != left_arm_res.error_code_.SUCCESS ||
        right_arm_res.error_code_.val != right_arm_res.error_code_.SUCCESS) {
      ROS_ERROR("Could not compute plan successfully");
      return;
    }
    moveit_msgs::MotionPlanResponse left_arm_planning_msg,
        right_arm_planning_msg;
    left_arm_res.getMessage(left_arm_planning_msg);
    right_arm_res.getMessage(right_arm_planning_msg);
    // Visualization
    moveit_msgs::DisplayTrajectory display_trajectory;
    // ros::Publisher display_publisher =
    //     handle_.advertise<moveit_msgs::DisplayTrajectory>(
    //         "/move_group/display_planned_path", 1, true);
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();
    display_trajectory.trajectory.push_back(left_arm_planning_msg.trajectory);
    display_trajectory.trajectory.push_back(right_arm_planning_msg.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.front(),
                                       left_arm_model_group_);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(),
                                       right_arm_model_group_);
    // Update the robot states
    robot_state_->setJointGroupPositions(
        left_arm_model_group_,
        left_arm_planning_msg.trajectory.joint_trajectory.points.back()
            .positions);
    robot_state_->setJointGroupPositions(
        right_arm_model_group_,
        right_arm_planning_msg.trajectory.joint_trajectory.points.back()
            .positions);
    planning_scene_->setCurrentState(*robot_state_.get());
    // Visualization
    visual_tools.publishRobotState(planning_scene_->getCurrentStateNonConst(),
                                   rviz_visual_tools::GREEN);
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to continue");
  }
  // void planning(G1ArmEnum arm_id, const Eigen::Isometry3d &target_pose);

 public:
  const moveit::core::RobotModelPtr &getKinematicModel() const {
    return robot_model_;
  }
  const moveit::core::RobotStatePtr &getKinematicState() const {
    return robot_state_;
  }
  const moveit::core::JointModelGroup *getArmModelGroup(
      G1ArmEnum arm_id) const {
    switch (arm_id) {
      case (G1ArmEnum::LeftArm):
        return left_arm_model_group_;
      case (G1ArmEnum::RightArm):
        return right_arm_model_group_;
    }
  }
  const moveit::core::JointModelGroup *getLeftArmModelGroup(
      G1ArmEnum arm_id) const {
    return left_arm_model_group_;
  }
  const moveit::core::JointModelGroup *getRightArmModelGroup(
      G1ArmEnum arm_id) const {
    return right_arm_model_group_;
  }

 protected:
  ros::NodeHandle handle_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  moveit::core::JointModelGroup *left_arm_model_group_, *right_arm_model_group_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_interface::PlannerManagerPtr planner_manager_;
  Eigen::Matrix<double, 14, 2> joint_limit_;
};
}  // namespace g1_dual_arm
