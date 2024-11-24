cmake_minimum_required(VERSION 3.5)
project(gnss_fault_injection)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gnss_fault_injection src/gnss_fault_injection.cpp)

ament_target_dependencies(gnss_fault_injection
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  gnss_fault_injection
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
