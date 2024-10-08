#include <switch_pcd_map/switch_pcd_map.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options;
  auto switch_map_system = std::make_shared<SwitchMapSystem>(options);

  executor.add_node(switch_map_system);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}