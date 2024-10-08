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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(switch_map_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)


set(dependencies
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  EIGEN3
  PCL
  switch_map_interfaces
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

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "pcl_ros::PCDPublisher"
  EXECUTABLE pcd_to_pointcloud
)

### Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()



# cmake_minimum_required(VERSION 3.5)
# project(switch_pcd_map)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
# endif()

# ## Find system dependencies
# find_package(Eigen3 REQUIRED)
# find_package(PCL REQUIRED QUIET COMPONENTS common io)

# ## Find ROS package dependencies
# find_package(ament_cmake REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# set(dependencies
#   pcl_conversions
#   rclcpp
#   rclcpp_components
#   sensor_msgs
#   geometry_msgs
#   tf2
#   tf2_eigen
#   tf2_geometry_msgs
#   tf2_ros
#   EIGEN3
#   PCL
# )

# ### Tools
# #
# add_library(pcd_to_pointcloud_lib SHARED src/switch_pcd_map.cpp)
# target_link_libraries(pcd_to_pointcloud_lib
#   ${PCL_LIBRARIES})
# target_include_directories(pcd_to_pointcloud_lib PUBLIC
#   ${PCL_INCLUDE_DIRS})
# ament_target_dependencies(pcd_to_pointcloud_lib
#   rclcpp
#   rclcpp_components
#   sensor_msgs
#   pcl_conversions
#   switch_map_interfaces)

# rclcpp_components_register_node(pcd_to_pointcloud_lib
#   PLUGIN "pcl_ros::PCDPublisher"
#   EXECUTABLE pcd_to_pointcloud)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/SingleMap.srv"
# )

# #############
# ## Testing ##
# #############

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
# endif()

# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# install(
#   TARGETS
#     # pcl_ros_tf
#     pcd_to_pointcloud_lib
#     # pcl_ros_filters

#   EXPORT export_pcl_ros
#   RUNTIME DESTINATION bin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
# )

# install(DIRECTORY plugins samples
#   DESTINATION share/${PROJECT_NAME})

# # Export modern CMake targets
# ament_export_targets(export_pcl_ros HAS_LIBRARY_TARGET)

# ament_export_dependencies(${dependencies})

# ament_package()