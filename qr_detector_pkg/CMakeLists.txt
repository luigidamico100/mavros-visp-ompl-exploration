cmake_minimum_required(VERSION 2.8.3)
project(qr_detector_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  tf
  urdf
  xacro
  rosservice
  message_generation
  message_runtime
)


add_message_files(
 FILES
 qr_detection_msg.msg
)

add_service_files(
  FILES
  qr_position_service.srv
  activate_service.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

include_directories( ${catkin_INCLUDE_DIRS})

add_executable(qr_detector_exe src/qr_detector.cpp)
target_link_libraries(qr_detector_exe ${catkin_LIBRARIES} )
