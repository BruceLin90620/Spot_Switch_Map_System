#include <map_manager/map_manager.hpp>
#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapManager>();
    std::string current_position = "000_002";  // 初始位置

    while (rclcpp::ok()) {
        // 請求新任務
        std::cout << "Requesting task from position: " << current_position << std::endl;
        node->request_and_process_path(current_position);
        
        // 給系統時間處理請求
        // rclcpp::sleep_for(std::chrono::seconds(2));
        
        // 處理ROS事件
        rclcpp::spin_some(node);
        
        // 如果沒有收到任務或任務已完成，等待5秒後重試
        if (!node->has_active_task()) {
            RCLCPP_INFO(node->get_logger(), "No active task. Waiting 5 seconds before retrying...");
            rclcpp::sleep_for(std::chrono::seconds(5));
        }
    }
    
    rclcpp::shutdown();
    return 0;
}