cmake_minimum_required(VERSION 3.0.2)
project(joint_to_cartesian_trajectory)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  kdl_parser
  tf2_eigen
  trajectory_msgs
  cartesian_control_msgs
  message_generation
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  JointToCartesianTrajectory.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  trajectory_msgs
  cartesian_control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  kdl_parser
  tf2_eigen
  trajectory_msgs
  control_msgs
  cartesian_control_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_to_cartesian_service 
  src/joint_to_cartesian_service_node.cpp
  src/joint_to_cartesian_service.cpp
  src/joint_to_cartesian_converter.cpp  
)
add_dependencies(joint_to_cartesian_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_to_cartesian_service ${catkin_LIBRARIES} ${GCC_COVERAGE_LINK_FLAGS})