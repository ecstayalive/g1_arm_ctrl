cmake_minimum_required(VERSION 3.0.2)

project(g1_description)

find_package(catkin REQUIRED urdf roscpp std_msgs tf genmsg)

catkin_package(CATKIN_DEPENDS)

include_directories(
	${Boost_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)

# foreach(dir config launch meshes urdf)
# install(DIRECTORY ${dir}/
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)
