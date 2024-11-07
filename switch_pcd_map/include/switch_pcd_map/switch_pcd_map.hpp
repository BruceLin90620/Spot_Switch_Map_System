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
#include <map>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "switch_map_interfaces/srv/single_map.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

struct TagPose {
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;
};

class SwitchMapSystem : public rclcpp::Node
{
public:
    explicit SwitchMapSystem(const rclcpp::NodeOptions & options);

protected:
    // Map switching related members
    std::string tf_frame_;
    std::vector<std::string> file_names_;
    int current_map_id_;
    sensor_msgs::msg::PointCloud2 cloud_;
    std::string cloud_topic_;
    size_t period_ms_;
    
    // Publishers and timer
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_pub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> pose_pub_;
    rclcpp::TimerBase::SharedPtr cloud_timer_;
    
    // Service
    rclcpp::Service<switch_map_interfaces::srv::SingleMap>::SharedPtr service_;
    
    // AprilTag localization related members
    std::string tags_poses_file_;
    std::map<std::string, std::map<int, TagPose>> all_tag_poses_;
    std::string current_area_;
    std::map<int, std::string> area_mapping_;
    
    // TF related members
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Member functions
    void publishCloud();
    void checkAndPublishPose();
    void switchMapCallback(
        const std::shared_ptr<switch_map_interfaces::srv::SingleMap::Request> request,
        std::shared_ptr<switch_map_interfaces::srv::SingleMap::Response> response);
    
    // Helper functions
    void loadMapPaths(const std::string& config_path);
    void loadAllTagPoses();
    void setupTF();
    void broadcastTagTransforms();
    std::pair<int, double> findVisibleTags();
    std::optional<geometry_msgs::msg::Pose> processDetection();
    
    // Transform helper functions
    geometry_msgs::msg::TransformStamped multiplyTransforms(
        const geometry_msgs::msg::Transform& t1,
        const geometry_msgs::msg::Transform& t2);
    Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::Transform& transform);
    geometry_msgs::msg::Transform matrixToTransform(const Eigen::Matrix4d& matrix);
    geometry_msgs::msg::Transform invertTransform(const geometry_msgs::msg::Transform& transform);
};

#endif // SWITCH_PCD_MAP_HPP