#ifndef SWITCH_PCD_MAP_HPP
#define SWITCH_PCD_MAP_HPP

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "switch_map_interfaces/srv/single_map.hpp"

class SwitchMapSystem : public rclcpp::Node
{
protected:
    std::string tf_frame_;
    std::vector<std::string> file_names_;
    int current_map_id_;

public:
    explicit SwitchMapSystem(const rclcpp::NodeOptions & options);

    sensor_msgs::msg::PointCloud2 cloud_;

    std::string cloud_topic_;
    size_t period_ms_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<switch_map_interfaces::srv::SingleMap>::SharedPtr service_;

    void publish();
    void switchMapCallback(const std::shared_ptr<switch_map_interfaces::srv::SingleMap::Request> request, 
                           std::shared_ptr<switch_map_interfaces::srv::SingleMap::Response> response);

private:
    void loadMapPaths(const std::string& config_path);
};

#endif // SWITCH_PCD_MAP_HPP