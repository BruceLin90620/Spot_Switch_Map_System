cmake_minimum_required(VERSION 3.8)
project(map_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(switch_map_interfaces REQUIRED)
find_package(routing_agent_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  switch_map_interfaces
  routing_agent_interfaces
  geometry_msgs
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# Build pose_utils library
add_library(pose_utils SHARED
  src/pose_utils.cpp
)

target_include_directories(pose_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pose_utils
  ${dependencies}
)

# Build the main library
add_library(${PROJECT_NAME} SHARED
  src/map_manager.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  pose_utils
  yaml-cpp
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# Build the executable
add_executable(map_manager_node src/map_manager_node.cpp)

target_link_libraries(map_manager_node
  ${PROJECT_NAME}
  pose_utils
)

ament_target_dependencies(map_manager_node
  ${dependencies}
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  pose_utils
  map_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} pose_utils)

ament_package()