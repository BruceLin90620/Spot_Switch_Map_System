#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP

#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include "switch_map_interfaces/srv/single_map.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class MapManager : public rclcpp::Node
{
protected:
    std::string tf_frame_;
    std::vector<std::string> file_names_;
    

public:
    explicit MapManager(const std::string& config_file_path);

    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    rclcpp::Client<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_map_client_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    int current_map_id_;

    void process_input(const std::vector<std::string>& input_vector);
    void switch_map(int new_map_id);
    void send_goal(int goal_id);
    void send_initial_pose(int goal_id);

    void load_config();
    // std::map<std::string, int> maps_;
    // geometry_msgs::msg::Pose last_odom_pose_;
    // explicit SwitchMapSystem(const rclcpp::NodeOptions & options);

    // sensor_msgs::msg::PointCloud2 cloud_;

    // std::string cloud_topic_;
    // size_t period_ms_;

    // std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;
    // rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Service<switch_map_interfaces::srv::SingleMap>::SharedPtr service_;

    // void publish();
    // void switchMapCallback(const std::shared_ptr<switch_map_interfaces::srv::SingleMap::Request> request, 
    //                        std::shared_ptr<switch_map_interfaces::srv::SingleMap::Response> response);

private:
    // void loadMapPaths(const std::string& config_path);
    YAML::Node config_;
    std::string config_file_path_;
};

#endif 