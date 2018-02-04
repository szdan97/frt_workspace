cmake_minimum_required(VERSION 2.8.3)
project(frt_loc_and_mapping)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  frt_custom_msgs
  roscpp
  rospy
  std_msgs
)

include_directories(include /home/daniel/frt_workspace/devel/include)

catkin_package(
  CATKIN_DEPENDS frt_custom_msgs roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ekf_slam_node src/ekf_slam.cpp)
target_link_libraries(ekf_slam_node ${catkin_LIBRARIES})
add_dependencies(ekf_slam_node frt_custom_msgs_generate_messages_cpp)

add_executable(local_nnsf_data_association_node src/local_nnsf_data_association_node.cpp)
target_link_libraries(local_nnsf_data_association_node ${catkin_LIBRARIES})
add_dependencies(local_nnsf_data_association_node frt_custom_msgs_generate_messages_cpp)