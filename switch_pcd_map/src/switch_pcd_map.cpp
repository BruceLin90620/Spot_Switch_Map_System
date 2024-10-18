#include <switch_pcd_map/switch_pcd_map.hpp>

SwitchMapSystem::SwitchMapSystem(const rclcpp::NodeOptions & options)
: rclcpp::Node("switch_pcd_map", options), tf_frame_("map"), current_map_id_(0)
{
    cloud_topic_ = "/pointcloud_map";
    tf_frame_ = this->declare_parameter("tf_frame", tf_frame_);
    period_ms_ = this->declare_parameter("publishing_period_ms", 3000);
    
    std::string config_path = this->declare_parameter<std::string>("config_path", "/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/switch_pcd_map/map_data/map_path.yaml");
    loadMapPaths(config_path);

    if (pcl::io::loadPCDFile(file_names_[current_map_id_], cloud_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "failed to open PCD file");
        throw std::runtime_error{"could not open pcd file"};
    }
    cloud_.header.frame_id = tf_frame_;

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms_),
        [this]() {
        this->publish();
        });

    // Create the service
    service_ = this->create_service<switch_map_interfaces::srv::SingleMap>(
        "switch_map",
        std::bind(&SwitchMapSystem::switchMapCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "PCD Publisher node initialized with service 'switch_map'");
}

void SwitchMapSystem::publish()
{
    cloud_.header.stamp = this->get_clock()->now();
    pub_->publish(cloud_);
}

void SwitchMapSystem::switchMapCallback(const std::shared_ptr<switch_map_interfaces::srv::SingleMap::Request> request,
                                            std::shared_ptr<switch_map_interfaces::srv::SingleMap::Response> response)
{
    int64_t requested_map_id = request->mapid;

    if (requested_map_id < 0 || static_cast<size_t>(requested_map_id) >= file_names_.size()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid map ID: %ld", requested_map_id);
        response->success = false;
        return;
    }

    if (requested_map_id == current_map_id_) {
        RCLCPP_INFO(this->get_logger(), "Already using map with ID: %ld", requested_map_id);
        response->success = true;
        return;
    }

    if (pcl::io::loadPCDFile(file_names_[requested_map_id], cloud_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", file_names_[requested_map_id].c_str());
        response->success = false;
    } else {
        cloud_.header.frame_id = tf_frame_;
        current_map_id_ = requested_map_id;
        RCLCPP_INFO(this->get_logger(), "Switched to PCD file: %s", file_names_[requested_map_id].c_str());
        response->success = true;
    }
}

void SwitchMapSystem::loadMapPaths(const std::string& config_path)
{
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["map_paths"]) {
            file_names_ = config["map_paths"].as<std::vector<std::string>>();
        } else {
            RCLCPP_ERROR(this->get_logger(), "No 'map_paths' key found in the configuration file");
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading YAML file: %s", e.what());
    }
}