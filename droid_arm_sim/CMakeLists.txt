cmake_minimum_required(VERSION 3.10)
project(droid_arm_sim)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  unitree_legged_msgs
  dual_arm_as
  g1_dual_arm_moveit_config
  droid_arm_ctrl
)

# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a exec_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to############
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  unitree_legged_msgs
  dual_arm_as
  droid_arm_ctrl

  # moveit_core
  # moveit_visual_tools
  # moveit_ros_planning_interface
  # DEPENDS EIGEN3
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${dual_arm_as_EXPORTED_TARGETS} ${unitree_legged_msgs_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
# scripts/my_python_script
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark executables for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h"
# PATTERN ".svn" EXCLUDE
# )

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# # myfile1
# # myfile2
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# ############
# # Testing ##
# ############

# # Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_droid_arm_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
