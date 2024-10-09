#include <map_manager/map_manager.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapManager>("/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/map_manager/entry_points_data/entry_points.yaml");
    
    std::vector<std::string> input = {"01", "10", "11", "20", "21", "00"};
    node->process_input(input);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}