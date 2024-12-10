#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "switch_map_interfaces/srv/single_map.hpp"
#include "switch_map_interfaces/srv/send_goal_pose.hpp"
#include "routing_agent_interfaces/srv/nav_service_msg.hpp"


// Manages robot navigation and map switching operations
class MapManager : public rclcpp::Node {
public:
    // Core functionality
    explicit MapManager();
    void request_and_process_path(std::string& current_position);
    bool has_active_task() const { return !path_sequence_.empty(); }
    std::string get_next_position() const;

private:
    // Navigation functions
    void switch_map(int new_map_id, const std::string& node_id);
    void send_goal( const std::string& node_id, const nlohmann::json& node_info, 
                    const std::string& current_position);
    void handle_send_failure();

    // Path processing
    void process_navigation_response(
        const routing_agent_interfaces::srv::NavServiceMsg::Response::SharedPtr& result, 
        std::string& current_position);
    void update_graph_information(const nlohmann::json& new_graph);
    void log_path_sequence();
    void process_next_point(std::string& current_position);
    bool process_map_switch_response(
        std::shared_future<switch_map_interfaces::srv::SingleMap::Response::SharedPtr> result_future,
        std::shared_future<switch_map_interfaces::srv::SingleMap::Response::SharedPtr> spot_result_future,
        int new_map_id);
    void prepare_goal_pose(
        switch_map_interfaces::srv::SendGoalPose::Request::SharedPtr& request,
        const nlohmann::json& node_info,
        const std::string& current_position,
        const std::string& node_id);

    // State variables
    std::string current_position_;
    int current_map_id_{0};
    std::vector<std::string> path_sequence_;
    size_t current_sequence_index_{0};
    nlohmann::json current_graph_;

    // ROS 2 communication
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Client<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_map_client_;
    rclcpp::Client<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_spot_map_client_;
    rclcpp::Client<switch_map_interfaces::srv::SendGoalPose>::SharedPtr send_goal_pose_client_;
    rclcpp::Client<routing_agent_interfaces::srv::NavServiceMsg>::SharedPtr nav_service_client_;
};

#endif // MAP_MANAGER_HPP