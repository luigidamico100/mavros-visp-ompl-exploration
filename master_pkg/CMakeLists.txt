cmake_minimum_required(VERSION 2.8.3)
project(master_pkg)

find_package(Eigen3 REQUIRED)
include_directories(/usr/local/include/eigen3/)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  px4_planner
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})

add_executable(master_exe src/master.cpp)
target_link_libraries(master_exe ${catkin_LIBRARIES})

add_executable(tf_map2base src/tf_map2base.cpp)
target_link_libraries(tf_map2base ${catkin_LIBRARIES})


