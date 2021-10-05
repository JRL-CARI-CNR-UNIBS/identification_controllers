cmake_minimum_required(VERSION 3.0.2)
project(multisine_estimation_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  cnr_hardware_interface
  joint_limits_interface
  name_sorting
  roscpp
  sensor_msgs
  std_msgs
  frequency_identification
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multisine_estimation_controllers
  CATKIN_DEPENDS cnr_controller_interface cnr_hardware_interface joint_limits_interface name_sorting roscpp sensor_msgs std_msgs frequency_identification
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
 src/${PROJECT_NAME}/multisine_estimation_controllers.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
 ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new
        $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
  target_precompile_headers(${PROJECT_NAME} PUBLIC
    <cmath>
    <Eigen/Core>
    <cnr_logger/cnr_logger.h>
    <ros/time.h>
    <sensor_msgs/JointState.h>
  )
endif()
