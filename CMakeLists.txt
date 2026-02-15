cmake_minimum_required(VERSION 3.0.2)
project(SimDataset)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  # rclcpp
  sensor_msgs
  std_msgs
  tf
  cv_bridge
)

find_package(Eigen3  REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_env
 CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(circle src/circle.cpp)

target_link_libraries(circle
  ${catkin_LIBRARIES}
)

add_executable(teleop_turtle_key_my src/teleop_turtle_key_my.cpp)
target_link_libraries(teleop_turtle_key_my
  ${catkin_LIBRARIES}
)

# add_executable(get_images src_save_image/save_all_images.cpp)
# target_link_libraries(get_images ${catkin_LIBRARIES})


add_executable(rostopic2tum src_save_image/rostopic2tum.cpp)
target_link_libraries(rostopic2tum ${catkin_LIBRARIES})
