#include <switch_pcd_map/switch_pcd_map.hpp>
#include <optional>
#include <tf2_eigen/tf2_eigen.hpp>

SwitchMapSystem::SwitchMapSystem(const rclcpp::NodeOptions & options)
: rclcpp::Node("switch_pcd_map", options),
  tf_frame_("map"),
  current_map_id_(0),
  current_area_("5152")
{
    // Initialize parameters
    cloud_topic_ = "/pointcloud_map";
    tf_frame_ = this->declare_parameter("tf_frame", tf_frame_);
    period_ms_ = this->declare_parameter("publishing_period_ms", 3000);
    
    tags_poses_file_ = this->declare_parameter<std::string>(
        "tags_poses_file",
        "/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/config/tags_position.yaml"
    );
    
    
    // Load configurations
    loadMapPaths(this->declare_parameter<std::string>(
        "config_path",
        "/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/config/map_path.yaml"
    ));
    loadAllTagPoses();
    
    // Load initial map
    if (pcl::io::loadPCDFile(file_names_[current_map_id_], cloud_) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to open PCD file");
        throw std::runtime_error{"Could not open PCD file"};
    }
    cloud_.header.frame_id = tf_frame_;

    // Set up publishers
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()
    );
    
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        10
    );

    // Set up timers
    cloud_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms_),
        std::bind(&SwitchMapSystem::publishCloud, this)
    );
    
    // Set up service
    service_ = this->create_service<switch_map_interfaces::srv::SingleMap>(
        "switch_map",
        std::bind(&SwitchMapSystem::switchMapCallback, this, 
                  std::placeholders::_1, std::placeholders::_2)
    );

    // Setup TF
    setupTF();
    
    RCLCPP_INFO(get_logger(), "Switch Map System initialized");
}

void SwitchMapSystem::publishCloud()
{
    cloud_.header.stamp = this->now();
    cloud_pub_->publish(cloud_);
}

void SwitchMapSystem::switchMapCallback(
    const std::shared_ptr<switch_map_interfaces::srv::SingleMap::Request> request,
    std::shared_ptr<switch_map_interfaces::srv::SingleMap::Response> response)
{
    int64_t requested_map_id = request->mapid;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (requested_map_id < 0 || static_cast<size_t>(requested_map_id) >= file_names_.size()) {
        RCLCPP_ERROR(get_logger(), "Invalid map ID: %ld", requested_map_id);
        response->success = false;
        return;
    }
    // TODO check if it is in the same map
    // Update current map ID and area
    current_map_id_ = requested_map_id;
    auto area_it = area_mapping_.find(requested_map_id);
    if (area_it != area_mapping_.end()) {
        current_area_ = area_it->second;
        RCLCPP_INFO(get_logger(), "Switching to area: %s", current_area_.c_str());
        broadcastTagTransforms();
    }

    // Load and publish new map
    if (pcl::io::loadPCDFile(file_names_[current_map_id_], cloud_) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load PCD file: %s",
                     file_names_[current_map_id_].c_str());
        response->success = false;
        return;
    }

    cloud_.header.frame_id = tf_frame_;
    publishCloud();
    
    // Wait for 1 second to allow map to be processed
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Try to publish initial pose using AprilTag
    auto pose = processDetection();
    if (pose.has_value()) {  // 使用 has_value() 來檢查是否有值
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose = pose.value();  // 使用 value() 來獲取值
        pose_pub_->publish(pose_msg);
        RCLCPP_INFO(get_logger(), "Published initial pose using AprilTag");
    } else {
        RCLCPP_WARN(get_logger(), "Could not determine initial pose using AprilTag");
    }
    
    RCLCPP_INFO(get_logger(), "Switched to PCD file: %s", 
                file_names_[current_map_id_].c_str());
    response->success = true;
}

void SwitchMapSystem::loadMapPaths(const std::string& config_path)
{
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["map_paths"]) {
            // First get the original paths
            auto original_paths = config["map_paths"].as<std::vector<std::string>>();
            
            // Clear existing file names
            file_names_.clear();
            
            // Add suffix to each path and store in file_names_
            for (const auto& path : original_paths) {
                file_names_.push_back(path + "/point_cloud.pcd");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "No 'map_paths' key found in the configuration file");
            throw std::runtime_error{"Invalid configuration file format"};
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Error reading YAML file: %s", e.what());
        throw;
    }
}

// 載入所有Tag的位置資訊
void SwitchMapSystem::loadAllTagPoses()
{
    try {
        YAML::Node config = YAML::LoadFile(tags_poses_file_);
        
        // Clear existing mappings
        area_mapping_.clear();
        int area_index = 0;

        // Iterate through all top-level entries (area names)
        for (const auto& area_it : config) {
            std::string area_name = area_it.first.as<std::string>();
            
            // Add to area mapping
            area_mapping_[area_index] = area_name;
            area_index++;

            // Load tag poses for this area
            std::map<int, TagPose> area_tags;
            for (const auto& tag_it : area_it.second) {
                int tag_id = std::stoi(tag_it.first.as<std::string>());
                auto pose_node = tag_it.second;
                
                TagPose tag_pose;
                // Read position
                tag_pose.position.x = pose_node["position"]["x"].as<double>();
                tag_pose.position.y = pose_node["position"]["y"].as<double>();
                tag_pose.position.z = pose_node["position"]["z"].as<double>();
                
                // Read orientation
                tag_pose.orientation.x = pose_node["orientation"]["x"].as<double>();
                tag_pose.orientation.y = pose_node["orientation"]["y"].as<double>();
                tag_pose.orientation.z = pose_node["orientation"]["z"].as<double>();
                tag_pose.orientation.w = pose_node["orientation"]["w"].as<double>();
                
                area_tags[tag_id] = tag_pose;
            }
            
            all_tag_poses_[area_name] = area_tags;
        }

        RCLCPP_INFO(get_logger(), "Successfully loaded tag poses and created area mapping:");
        for (const auto& [index, area_name] : area_mapping_) {
            RCLCPP_INFO(get_logger(), "Area index %d: %s", index, area_name.c_str());
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error loading tag poses: %s", e.what());
        throw;
    }
}

// 設置TF相關組件
void SwitchMapSystem::setupTF()
{
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// 廣播Tag的TF
void SwitchMapSystem::broadcastTagTransforms()
{
    auto current_tags = all_tag_poses_[current_area_];
    for (const auto& [tag_id, tag_pose] : current_tags) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "tag_" + std::to_string(tag_id);
        
        transform.transform.translation.x = tag_pose.position.x;
        transform.transform.translation.y = tag_pose.position.y;
        transform.transform.translation.z = tag_pose.position.z;
        
        transform.transform.rotation = tag_pose.orientation;
        
        static_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(get_logger(), "Broadcasting transform for tag %d in %s at %f",
                     tag_id, current_area_.c_str(), transform.transform.translation.y);
    }
}

std::pair<int, double> SwitchMapSystem::findVisibleTags()
{
    try {
        // 獲取所有的frame
        std::string frames = tf_buffer_->allFramesAsString();
        // RCLCPP_INFO(get_logger(), "Current TF frames:\n%s", frames.c_str());

        // 尋找所有fiducial frames
        std::vector<std::string> fiducial_frames;
        std::istringstream iss(frames);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.find("fiducial_") != std::string::npos && 
                line.find("filtered") == std::string::npos) {
                size_t start = line.find("fiducial_");
                size_t end = line.find(" ", start);
                if (end == std::string::npos) end = line.length();
                fiducial_frames.push_back(line.substr(start, end - start));
            }
        }
        
        if (fiducial_frames.empty()) {
            return {-1, 0.0};
        }
        
        // 計算到每個可見tag的距離
        std::map<int, double> visible_tags;
        rclcpp::Time now = this->now();

        for (const auto& frame : fiducial_frames) {
            try {
                // 從frame名稱中提取tag ID
                int tag_id = std::stoi(frame.substr(9));

                // if (!tf_buffer_->canTransform("map", frame, now ,
                //     rclcpp::Duration::from_seconds(1))) {
                //     RCLCPP_INFO(get_logger(), "Transform not currently available for tag %s", frame);
                //     continue;
                // }
                if (tf_buffer_->canTransform("map", frame, tf2::TimePointZero)){

                    geometry_msgs::msg::TransformStamped transform;
                    try {
                        // 先嘗試獲取最新的 transform
                        transform = tf_buffer_->lookupTransform(
                            "body", frame, tf2::TimePointZero
                        );
                    } catch (const tf2::TransformException& e) {
                        RCLCPP_INFO(get_logger(), "Transform not available for tag %d: %s", 
                                tag_id, e.what());
                        continue;
                    }

                    // 檢查時間差
                    rclcpp::Time transform_time(transform.header.stamp);
                    double time_diff = (now - transform_time).seconds();

                    if (time_diff > 0.5) {  // 如果數據超過0.5秒沒有更新
                        RCLCPP_INFO(get_logger(), "Transform for tag %d is too old (%.2f seconds)", 
                                tag_id, time_diff);
                        continue;
                    }
                    
                    // 計算距離
                    double distance = std::sqrt(
                        std::pow(transform.transform.translation.x, 2) +
                        std::pow(transform.transform.translation.y, 2) +
                        std::pow(transform.transform.translation.z, 2)
                    );
                    
                    visible_tags[tag_id] = distance;
                    RCLCPP_INFO(get_logger(), "Added tag %d at distance %.2f meters (age: %.2f seconds)", 
                            tag_id, distance, time_diff);
                }

            } catch (const std::exception& e) {
                RCLCPP_INFO(get_logger(), "Error processing tag %d: %s",
                        std::stoi(frame.substr(9)), e.what());
                continue;
            }
        }
        
        if (visible_tags.empty()) {
            RCLCPP_INFO(get_logger(), "No valid transforms found");
            return {-1, 0.0};
        }

        RCLCPP_INFO(get_logger(), "All visible tags:");
        for(const auto& [tag_id, dist] : visible_tags) {
            RCLCPP_INFO(get_logger(), "Tag %d: %.2f meters", tag_id, dist);
        }
        
        auto nearest = std::min_element(
            visible_tags.begin(), visible_tags.end(),
            [](const auto& p1, const auto& p2) { return p1.second < p2.second; }
        );
        
        RCLCPP_INFO(get_logger(), "Selected nearest tag: %d at %.2f meters", 
                    nearest->first, nearest->second);
        return {nearest->first, nearest->second};
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error finding visible tags: %s", e.what());
        return {-1, 0.0};
    }
}

// 處理檢測並計算位姿
std::optional<geometry_msgs::msg::Pose> SwitchMapSystem::processDetection()
{
    try {
        // 找到最近的可見tag
        auto [nearest_tag_id, distance] = findVisibleTags();
        if (nearest_tag_id < 0) {
            RCLCPP_WARN(get_logger(), "No AprilTags visible");
            return std::nullopt;
        }
        
        // 檢查tag是否在當前區域的信息中
        auto current_tags = all_tag_poses_[current_area_];
        if (current_tags.find(nearest_tag_id) == current_tags.end()) {
            RCLCPP_WARN(get_logger(), "Tag %d not in current area (%s) info",
                        nearest_tag_id, current_area_.c_str());
            return std::nullopt;
        }

        // 獲取必要的transforms
        geometry_msgs::msg::TransformStamped base_to_tag = 
            tf_buffer_->lookupTransform(
                "body",
                "fiducial_" + std::to_string(nearest_tag_id),
                tf2::TimePointZero
            );
        
        geometry_msgs::msg::TransformStamped map_to_tag = 
            tf_buffer_->lookupTransform(
                "map",
                "tag_" + std::to_string(nearest_tag_id),
                tf2::TimePointZero
            );
        
        // 計算機器人在地圖中的位姿
        auto robot_pose_map = multiplyTransforms(
            map_to_tag.transform,
            invertTransform(base_to_tag.transform)
        );
        
        geometry_msgs::msg::Pose pose;
        pose.position.x = robot_pose_map.transform.translation.x;  // 添加 .transform
        pose.position.y = robot_pose_map.transform.translation.y;
        pose.position.z = robot_pose_map.transform.translation.z;
        pose.orientation = robot_pose_map.transform.rotation;  // 添加 .transform
        
        RCLCPP_INFO(get_logger(),
            "Computed pose using tag %d at distance %.2f m: x=%.2f, y=%.2f, z=%.2f",
            nearest_tag_id, distance,
            pose.position.x, pose.position.y, pose.position.z);
        
        return pose;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
        return std::nullopt;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to compute transform: %s", e.what());
        return std::nullopt;
    }
}

// Transform相關的輔助函數
geometry_msgs::msg::Transform SwitchMapSystem::invertTransform(
    const geometry_msgs::msg::Transform& transform)
{
    Eigen::Matrix4d mat = transformToMatrix(transform);
    Eigen::Matrix4d inv_mat = mat.inverse();
    return matrixToTransform(inv_mat);
}

Eigen::Matrix4d SwitchMapSystem::transformToMatrix(
    const geometry_msgs::msg::Transform& transform)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    
    // 設置旋轉部分
    Eigen::Quaterniond q(
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z
    );
    mat.block<3,3>(0,0) = q.toRotationMatrix();
    
    // 設置平移部分
    mat(0,3) = transform.translation.x;
    mat(1,3) = transform.translation.y;
    mat(2,3) = transform.translation.z;
    
    return mat;
}

geometry_msgs::msg::Transform SwitchMapSystem::matrixToTransform(
    const Eigen::Matrix4d& matrix)
{
    geometry_msgs::msg::Transform transform;
    
    // 提取平移部分
    transform.translation.x = matrix(0,3);
    transform.translation.y = matrix(1,3);
    transform.translation.z = matrix(2,3);
    
    // 提取旋轉部分並轉換為四元數
    Eigen::Matrix3d rot = matrix.block<3,3>(0,0);
    Eigen::Quaterniond q(rot);
    transform.rotation.w = q.w();
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    
    return transform;
}

geometry_msgs::msg::TransformStamped SwitchMapSystem::multiplyTransforms(
    const geometry_msgs::msg::Transform& t1,
    const geometry_msgs::msg::Transform& t2)
{
    Eigen::Matrix4d m1 = transformToMatrix(t1);
    Eigen::Matrix4d m2 = transformToMatrix(t2);
    Eigen::Matrix4d result = m1 * m2;
    
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.transform = matrixToTransform(result);
    return transform_stamped;
}