#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include "switch_map_interfaces/srv/single_map.hpp"
#include "switch_map_interfaces/srv/send_goal_pose.hpp"
#include "routing_agent_interfaces/srv/nav_service_msg.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "pose_utils/pose_utils.hpp"
#include <nlohmann/json.hpp>

class MapManager : public rclcpp::Node {
public:
    explicit MapManager();
    void request_and_process_path(std::string& current_position);  // Changed to reference
    bool has_active_task() const { return !path_sequence_.empty(); }
    std::string get_next_position() const;

private:
    void switch_map(int new_map_id, const std::string& node_id);
    void send_goal(const std::string& node_id, const nlohmann::json& node_info, const std::string& current_position);
    void handle_send_failure();
    std::string current_position_;
    
    int current_map_id_{0};
    std::vector<std::string> path_sequence_;
    size_t current_sequence_index_{0};
    nlohmann::json current_graph_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Client<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_map_client_;
    rclcpp::Client<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_spot_map_client_;
    rclcpp::Client<switch_map_interfaces::srv::SendGoalPose>::SharedPtr send_goal_pose_client_;
    rclcpp::Client<routing_agent_interfaces::srv::NavServiceMsg>::SharedPtr nav_service_client_;
};

#endif // MAP_MANAGER_HPP