#include <switch_pcd_map/switch_pcd_map.hpp>

int main(int argc, char * argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create executor and node
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::NodeOptions options;
    auto switch_map_system = std::make_shared<SwitchMapSystem>(options);

    // Add node to executor and spin
    executor.add_node(switch_map_system);
    executor.spin();

    // Cleanup
    rclcpp::shutdown();
    return 0;
}