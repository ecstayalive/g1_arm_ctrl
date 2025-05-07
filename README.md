# Arm Controller
G1人形机器人控制包,该包使用ROS1控制G1人形机器人双机械臂
## Usage
1. 创建工作区:
```bash
mkdir -p ~/g1_arm_ws/src
cd ~/g1_arm_ws/src

```
2. 将该项目包放在工作区`src`目录下.
3. 编译项目:
```bash
cd ~/g1_arm_ws
catkin_make
source ./devel/setup.bash
```
4. 在运行本项目前需要开启人形机器人运动控制器,之后运行以下的命令:
```bash
roslaunch droid_arm_ctrl g1_arm_ctrl.launch direct_ctrl:=false
```
此时应能观察到机械臂机器人双臂先切换成阻尼模式,后抬起至初始位置.
运行之后,终端会输出控制器目前所能支持的全部动作库,例如:
```text
[INFO] [1745847730.666439593, 251.224000000]: Motion name: answer2
[INFO] [1745847730.666458573, 251.224000000]: Motion name: place
[INFO] [1745847730.666466263, 251.224000000]: Motion name: answer1
[INFO] [1745847730.666474433, 251.224000000]: Motion name: pickup
[INFO] [1745847730.666481853, 251.224000000]: Motion name: promotion
[INFO] [1745847730.666490093, 251.224000000]: Motion name: point
[INFO] [1745847730.666498123, 251.224000000]: Motion name: shrug
[INFO] [1745847730.666506453, 251.224000000]: Motion name: self_introduction
[INFO] [1745847730.666514983, 251.224000000]: Motion name: hello
[INFO] [1745847731.835773896, 251.850000000]: Controller started
```
调用时,新开一个终端:
```bash
cd ~/g1_arm_ws; source ./devel/setup.bash
rostopic pub --once /droid_arm_ctrl/arm_motion/goal dual_arm_as/ArmMotionActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  motion: '${MotionName}'"
```
# Additional Information
项目包中的`g1_description`修改自宇树开源项目`unitree_ros`
