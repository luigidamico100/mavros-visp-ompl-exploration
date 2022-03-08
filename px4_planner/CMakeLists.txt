cmake_minimum_required(VERSION 3.0.2)
project(px4_planner)

find_package(Eigen3 REQUIRED)
include_directories(/usr/local/include/eigen3/)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  kdl_parser
  eigen_conversions
  tf
  octomap_msgs
  octomap_ros
  nav_msgs
  actionlib_msgs
  rosservice
)

add_service_files(
	FILES
	planner_commander_service.srv
)



add_action_files(
  DIRECTORY action
  FILES Fly.action
)

generate_messages(
	DEPENDENCIES
	nav_msgs 
	std_msgs 
	actionlib_msgs 
	px4_planner 
	geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})


add_executable(fly_action_server src/fly_action_server_main.cpp src/fly_action_server.cpp)
target_link_libraries(fly_action_server ${catkin_LIBRARIES} )

add_executable(fly_action_client src/fly_action_client.cpp)
target_link_libraries(fly_action_client ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ompl fcl )





