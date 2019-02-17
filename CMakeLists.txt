cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-ignored-attributes")
project(teleop)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  std_msgs
  cmake_modules
  pluginlib
  wam_control
  user_interface
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES teleop
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(wam_teleop src/wam_teleop.cpp)
target_link_libraries(wam_teleop
  ${catkin_LIBRARIES}
  arm_control
  bhand_control
)
