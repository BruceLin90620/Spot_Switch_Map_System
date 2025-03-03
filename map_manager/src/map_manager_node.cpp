#include <map_manager/map_manager.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapManager>();

    // Set initial position
    std::string current_position = "000_019";

    while (rclcpp::ok()) {
        std::cout << "Requesting task from position: " << current_position << std::endl;
        node->request_and_process_path(current_position);
        
        rclcpp::spin_some(node);
        
        // If no active task is present, wait for 5 seconds before retrying
        if (!node->has_active_task()) {
            RCLCPP_INFO(node->get_logger(), "No active task. Waiting 5 seconds before retrying...");
            rclcpp::sleep_for(std::chrono::seconds(5));
        }
    }
    
    rclcpp::shutdown();
    return 0;
}