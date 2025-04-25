add_executable(test_arm_api test_arm_api.cpp)
target_link_libraries(test_arm_api unitree_sdk2)

add_executable(read_joint read_joint.cpp)
target_link_libraries(read_joint unitree_sdk2)
