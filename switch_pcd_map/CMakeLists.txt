cmake_minimum_required(VERSION 3.5)
project(switch_pcd_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

## Find system dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)

## Find ROS package dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(switch_map_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

set(dependencies
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  EIGEN3
  PCL
  switch_map_interfaces
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
)

### Build the library
add_library(${PROJECT_NAME} SHARED
  src/switch_pcd_map.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  yaml-cpp
)

### Build the executable
add_executable(switch_pcd_map_node src/switch_pcd_map_node.cpp)
target_link_libraries(switch_pcd_map_node
  ${PROJECT_NAME}
)

ament_target_dependencies(switch_pcd_map_node
  ${dependencies}
)

### Install
install(TARGETS ${PROJECT_NAME} switch_pcd_map_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


ament_package()