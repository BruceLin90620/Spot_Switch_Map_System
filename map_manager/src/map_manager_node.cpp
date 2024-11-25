#include <map_manager/map_manager.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MapManager>("/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/config/entry_points.yaml");

    // std::vector<std::string> input = {"10", "11", "12", "13", "20", "21", "00", "01", "02", "10"};
    // std::vector<std::string> input = {"10", "11", "12", "13", "20", "21", "00", "01", "02", "10", "02", "01", "00", "21", "20", "13", "12", "11", "10"};
    // std::vector<std::string> input = {"00", "01", "10", "11", "12", "13", "12", "11", "10", "01", "00"};
    // std::vector<std::string> input = {"00", "01", "10", "11", "12", "13", "30", "31", "41", "40", "21", "40", "41", "31", "30", "12", "11", "10", "01", "00"};
    std::vector<std::string> input = {"00", "01", "10", "11", "12", "13", "20", "21", "22", "23", "40", "23", "22", "21", "20", "13", "12", "11", "10", "01", "00"};

    node->process_input(input);
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}