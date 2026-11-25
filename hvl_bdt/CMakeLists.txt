cmake_minimum_required(VERSION 3.8)
project(hvl_bdt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # PCL is very slow in Debug
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation sample_consensus)

add_executable(hvl_bdt_node src/hvl_bdt_node.cpp)
target_include_directories(hvl_bdt_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(hvl_bdt_node ${PCL_LIBRARIES})
ament_target_dependencies(hvl_bdt_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs tf2_ros pcl_ros pcl_conversions)

install(TARGETS hvl_bdt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
