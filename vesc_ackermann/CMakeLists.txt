cmake_minimum_required(VERSION 3.5)
project(vesc_ackermann)

# Setting C++ standard to 14
if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# node executable
ament_auto_add_executable(vesc_to_odom_node
  src/vesc_to_odom_node.cpp
  src/vesc_to_odom.cpp
)

ament_auto_add_executable(ackermann_to_vesc_node
  src/ackermann_to_vesc_node.cpp
  src/ackermann_to_vesc.cpp
)

# components library
ament_auto_add_library(vesc_ackermann_nodelet
  src/ackermann_to_vesc_nodelet.cpp
  src/ackermann_to_vesc.cpp
  src/vesc_to_odom_nodelet.cpp
  src/vesc_to_odom.cpp
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
