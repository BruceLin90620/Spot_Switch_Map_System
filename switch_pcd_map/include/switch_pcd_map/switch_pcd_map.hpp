#ifndef SWITCH_PCD_MAP_HPP_
#define SWITCH_PCD_MAP_HPP_

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


// Structure to store AprilTag pose information
struct TagPose {
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Quaternion orientation;
};

/**
 * @brief Class handling PCD map switching and AprilTag-based localization
 * 
 * This system manages multiple point cloud maps and uses AprilTags for 
 * initial pose estimation when switching between maps.
 */
class SwitchMapNode : public rclcpp::Node 
{
public:
  explicit SwitchMapNode(const rclcpp::NodeOptions & options);

protected:
  // Map-related members
  std::string tf_frame_;                    // Transform frame for the map
  std::vector<std::string> file_names_;     // List of PCD file paths
  int current_map_id_;                      // Current active map ID
  sensor_msgs::msg::PointCloud2 cloud_;     // Current point cloud data
  std::string cloud_topic_;                 // Topic for publishing point cloud
  size_t period_ms_;                        // Publishing period in milliseconds
  
  // ROS2 communication members
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr cloud_timer_;
  rclcpp::Service<switch_map_interfaces::srv::SingleMap>::SharedPtr switch_map_service_;
  
  // AprilTag-related members
  std::string tags_poses_file_;             // Path to tag poses configuration
  std::map<std::string, std::map<int, TagPose>> all_tag_poses_;  // Tag poses by area
  std::string current_area_;                // Current area identifier
  std::map<int, std::string> area_mapping_; // Mapping of area IDs to names
  
  // TF2 members
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Core functionality methods
  void publish_cloud();
  void check_and_publish_pose();
  void handle_switch_map_request(
      const std::shared_ptr<switch_map_interfaces::srv::SingleMap::Request> request,
      std::shared_ptr<switch_map_interfaces::srv::SingleMap::Response> response);
  
  // Initialization and setup methods
  void load_map_paths(const std::string& config_path);
  void load_all_tag_poses();
  void setup_tf();
  void broadcast_tag_transforms();
  
  // AprilTag processing methods
  std::pair<int, double> find_visible_tags();
  std::optional<geometry_msgs::msg::Pose> process_detection();
  
  // Transform utility methods
  geometry_msgs::msg::TransformStamped multiply_transforms(
      const geometry_msgs::msg::Transform& t1,
      const geometry_msgs::msg::Transform& t2);
  Eigen::Matrix4d transform_to_matrix(const geometry_msgs::msg::Transform& transform);
  geometry_msgs::msg::Transform matrix_to_transform(const Eigen::Matrix4d& matrix);
  geometry_msgs::msg::Transform invert_transform(const geometry_msgs::msg::Transform& transform);
};

#endif  // SWITCH_PCD_MAP_HPP_