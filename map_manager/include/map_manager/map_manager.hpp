#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include "switch_map_interfaces/srv/single_map.hpp"
#include "switch_map_interfaces/srv/send_goal_pose.hpp"
#include "switch_map_interfaces/srv/goal_path.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "pose_utils/pose_utils.hpp"

class MapManager : public rclcpp::Node {
public:
    explicit MapManager(const std::string& config_file_path);
    void process_input(const std::vector<std::string>& input_vector);

private:
    void handle_goal_path(
        const std::shared_ptr<switch_map_interfaces::srv::GoalPath::Request> request,
        std::shared_ptr<switch_map_interfaces::srv::GoalPath::Response> response);
    void switch_map(int new_map_id, double x, double y);
    void send_goal(double x, double y);
    void load_config();
    void handle_send_failure();

    std::string config_file_path_;
    YAML::Node config_;
    int current_map_id_{0};
    std::vector<std::tuple<int, double, double>> path_sequence_; // tuple<map_id, x, y>
    size_t current_sequence_index_{0};
    
    // Service server
    rclcpp::Service<switch_map_interfaces::srv::GoalPath>::SharedPtr routing_path_service_;
    
    // Service clients
    rclcpp::Client<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_map_client_;
    rclcpp::Client<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_spot_map_client_;
    rclcpp::Client<switch_map_interfaces::srv::SendGoalPose>::SharedPtr send_goal_pose_client_;
};

#endif // MAP_MANAGER_HPP