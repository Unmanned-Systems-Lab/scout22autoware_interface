cmake_minimum_required(VERSION 3.8)
project(scout22autoware_interface)

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(scout_msgs REQUIRED)

ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

add_executable(AUTO2SCOUT src/Auto22scout_new.cpp)
ament_target_dependencies(AUTO2SCOUT 
rclcpp
autoware_auto_control_msgs
geometry_msgs
std_msgs )

add_executable(SCOUT2AUTO src/scout22Auto_new.cpp)
ament_target_dependencies(SCOUT2AUTO 
rclcpp
autoware_auto_vehicle_msgs
tier4_vehicle_msgs
scout_msgs
geometry_msgs )

install(TARGETS
SCOUT2AUTO
AUTO2SCOUT
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
