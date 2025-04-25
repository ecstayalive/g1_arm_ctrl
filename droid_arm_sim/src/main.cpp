#include "droid_arm_sim/arm_control.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_arm_sim_ctrl");
  ros::NodeHandle handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  g1_controller::G1ArmSimController g1_arm_controller(handle);
  g1_arm_controller.init();
  g1_arm_controller.start();
  ros::waitForShutdown();
  return 0;
}
