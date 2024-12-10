#include <switch_pcd_map/switch_pcd_map.hpp>

/**
 * @brief Main entry point for the switch map node
 * 
 * Initializes ROS2, creates and spins the SwitchMapNode
 */
int main(int argc, char * argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create executor and node options
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  // Create and add the switch map node
  auto node = std::make_shared<SwitchMapNode>(options);
  executor.add_node(node);
  
  // Spin until shutdown
  executor.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}