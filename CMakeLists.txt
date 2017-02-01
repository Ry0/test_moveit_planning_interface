cmake_minimum_required(VERSION 2.8.3)
project(test_moveit_planning_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_msgs
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  )

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  )

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(test_moveit_plan
  src/test_moveit_plan.cpp
  )

target_link_libraries(test_moveit_plan
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN_LIBRARIES}
  )

add_executable(test_add_object
  src/test_add_object.cpp
  )

target_link_libraries(test_add_object
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN_LIBRARIES}
  )