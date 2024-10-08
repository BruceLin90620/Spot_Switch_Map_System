#include <map_manager/map_manager.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<MapManager>();
    
    // executor.add_node(node);
    
    std::vector<std::string> input = {"02", "11", "12", "21"};
    node->process_input(input);
    
    // executor.spin();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}