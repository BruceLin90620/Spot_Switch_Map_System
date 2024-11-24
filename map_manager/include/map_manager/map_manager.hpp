#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include "switch_map_interfaces/srv/single_map.hpp"
#include "switch_map_interfaces/srv/send_goal_pose.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "pose_utils/pose_utils.hpp"

class MapManager : public rclcpp::Node {
public:
    explicit MapManager(const std::string& config_file_path);
    void process_input(const std::vector<std::string>& input_vector);

private:
    void switch_map(int new_map_id, int goal_id);
    void send_goal(int goal_id);
    void load_config();
    void handle_send_failure();
    std::pair<int, int> getNextPoint(size_t current_index) const;
    std::pair<int, int> getPreviousPoint(size_t current_index) const;

    std::string config_file_path_;
    YAML::Node config_;
    int current_map_id_{0};  // 初始化為1
    std::vector<std::pair<int, int>> path_sequence_;  // pair<map_id, point_id>
    size_t current_sequence_index_{0};

    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;

    // Service clients
    rclcpp::Client<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_map_client_;
    rclcpp::Client<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_spot_map_client_;
    rclcpp::Client<switch_map_interfaces::srv::SendGoalPose>::SharedPtr send_goal_pose_client_;
};

#endif // MAP_MANAGER_HPP