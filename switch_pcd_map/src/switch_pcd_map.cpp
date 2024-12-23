#include <switch_pcd_map/switch_pcd_map.hpp>


SwitchMapNode::SwitchMapNode(const rclcpp::NodeOptions & options):   
    Node("switch_pcd_map", options),
    tf_frame_("map")
{
    // Initialize parameters with default values
    cloud_topic_ = "/pointcloud_map";
    tf_frame_ = this->declare_parameter("tf_frame", tf_frame_);
    period_ms_ = this->declare_parameter("publishing_period_ms", 3000);
    current_map_id_ = this->declare_parameter("current_map_id", 0);
    current_area_ = this->declare_parameter<std::string>("current_area", "5152");
    
    tags_poses_file_ = this->declare_parameter<std::string>(
        "tags_poses_file",
        // "/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/switch_map_system_bringup/config/tags_position.yaml"
        "src/Spot_Switch_Map_System/switch_map_system_bringup/config/tags_position.yaml"
    );
    
    // Load configurations and initial map
    load_map_paths(this->declare_parameter<std::string>(
        "config_path",
        // "/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/switch_map_system_bringup/config/map_path.yaml"
        "src/Spot_Switch_Map_System/switch_map_system_bringup/config/map_path.yaml"
    ));
    load_all_tag_poses();
    
    // Load initial point cloud map
    if (pcl::io::loadPCDFile(file_names_[current_map_id_], cloud_) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to open PCD file");
        throw std::runtime_error{"Could not open PCD file"};
    }
    cloud_.header.frame_id = tf_frame_;

    // Initialize publishers
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()
    );
    
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        10
    );

    // Set up periodic publishing timer
    cloud_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms_),
        std::bind(&SwitchMapNode::publish_cloud, this)
    );
    
    // Initialize map switching service
    switch_map_service_ = this->create_service<switch_map_interfaces::srv::SingleMap>(
        "switch_map",
        std::bind(&SwitchMapNode::handle_switch_map_request, this, 
                    std::placeholders::_1, std::placeholders::_2)
    );

    setup_tf();
    RCLCPP_INFO(get_logger(), "Switch Map System initialized");
}

/**
 * @brief Publishes the current point cloud map
 */
void SwitchMapNode::publish_cloud()
{
    cloud_.header.stamp = this->now();
    cloud_pub_->publish(cloud_);
}

/**
 * @brief Service callback for handling map switch requests
 * 
 * Validates the request, loads the new map, and attempts to initialize pose using AprilTags
 */
void SwitchMapNode::handle_switch_map_request(
    const std::shared_ptr<switch_map_interfaces::srv::SingleMap::Request> request,
    std::shared_ptr<switch_map_interfaces::srv::SingleMap::Response> response)
{
    int64_t requested_map_id = request->mapid;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Validate map ID
    if (requested_map_id < 0 || static_cast<size_t>(requested_map_id) >= file_names_.size()) {
        RCLCPP_ERROR(get_logger(), "Invalid map ID: %ld", requested_map_id);
        response->success = false;
        return;
    }

    // Update current map and area information
    current_map_id_ = requested_map_id;
    auto area_it = area_mapping_.find(requested_map_id);
    if (area_it != area_mapping_.end()) {
        current_area_ = area_it->second;
        RCLCPP_INFO(get_logger(), "Switching to area: %s", current_area_.c_str());
        broadcast_tag_transforms();
    }

    // Load and publish new map
    if (pcl::io::loadPCDFile(file_names_[current_map_id_], cloud_) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load PCD file: %s",
                    file_names_[current_map_id_].c_str());
        response->success = false;
        return;
    }

    cloud_.header.frame_id = tf_frame_;
    publish_cloud();
    
    // Allow time for map processing
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Attempt initial pose estimation using AprilTags
    auto pose = process_detection();
    if (pose.has_value()) {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose = pose.value();
        pose_pub_->publish(pose_msg);
        RCLCPP_INFO(get_logger(), "Published initial pose using AprilTag");
    } else {
        RCLCPP_WARN(get_logger(), "Could not determine initial pose using AprilTag");
    }
    
    RCLCPP_INFO(get_logger(), "Switched to PCD file: %s", 
                file_names_[current_map_id_].c_str());
    response->success = true;
}


/**
 * @brief Load map paths from configuration file
 * 
 * @param config_path Path to the YAML configuration file
 * @throw std::runtime_error if configuration file is invalid or cannot be read
 */
void SwitchMapNode::load_map_paths(const std::string& config_path)
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


/**
 * @brief Load AprilTag poses for all areas from configuration file
 * 
 * @throw std::runtime_error if tag poses file is invalid or cannot be read
 */
void SwitchMapNode::load_all_tag_poses()
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


/**
 * @brief Initialize TF2 components
 */
void SwitchMapNode::setup_tf()
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


/**
 * @brief Broadcast transforms for all AprilTags in the current area
 */
void SwitchMapNode::broadcast_tag_transforms()
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
    RCLCPP_INFO(
      get_logger(),
      "Broadcasting transform for tag %d in %s at position (x: %f, y: %f, z: %f)",
      tag_id,
      current_area_.c_str(),
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z
    );
  }
}


/**
 * @brief Find visible AprilTags and their distances
 * 
 * @return Pair of <tag_id, distance>. Returns <-1, 0.0> if no tags are visible
 */
std::pair<int, double> SwitchMapNode::find_visible_tags()
{
  try {
    // Get all TF frames
    std::string frames = tf_buffer_->allFramesAsString();

    // Find all fiducial frames
    std::vector<std::string> fiducial_frames;
    std::istringstream iss(frames);
    std::string line;
    while (std::getline(iss, line)) {
      if (line.find("fiducial_") != std::string::npos && 
          line.find("filtered") == std::string::npos) {
        size_t start = line.find("fiducial_");
        size_t end = line.find(" ", start);
        if (end == std::string::npos) {
          end = line.length();
        }
        fiducial_frames.push_back(line.substr(start, end - start));
      }
    }
    
    if (fiducial_frames.empty()) {
      return {-1, 0.0};
    }
    
    // Calculate distance to each visible tag
    std::map<int, double> visible_tags;
    rclcpp::Time now = this->now();

    for (const auto& frame : fiducial_frames) {
      try {
        // Extract tag ID from frame name
        int tag_id = std::stoi(frame.substr(9));

        if (tf_buffer_->canTransform("map", frame, tf2::TimePointZero)) {
          geometry_msgs::msg::TransformStamped transform;
          try {
            transform = tf_buffer_->lookupTransform("body", frame, tf2::TimePointZero);
          } catch (const tf2::TransformException& e) {
            RCLCPP_INFO(get_logger(), "Transform not available for tag %d: %s", 
                      tag_id, e.what());
            continue;
          }

          // Check timestamp difference
          rclcpp::Time transform_time(transform.header.stamp);
          double time_diff = (now - transform_time).seconds();

          if (time_diff > 0.5) {
            RCLCPP_INFO(get_logger(), "Transform for tag %d is too old (%.2f seconds)", 
                      tag_id, time_diff);
            continue;
          }
          
          // Calculate distance
          double distance = std::sqrt(
            std::pow(transform.transform.translation.x, 2) +
            std::pow(transform.transform.translation.y, 2) +
            std::pow(transform.transform.translation.z, 2)
          );
          
          visible_tags[tag_id] = distance;
          RCLCPP_INFO(get_logger(), "Tag %d visible at %.2f meters (data age: %.2f seconds)", 
                    tag_id, distance, time_diff);
        }
      } catch (const std::exception& e) {
        RCLCPP_INFO(get_logger(), "Error processing tag %s: %s",
                  frame.c_str(), e.what());
        continue;
      }
    }
    
    if (visible_tags.empty()) {
      RCLCPP_INFO(get_logger(), "No valid transforms found");
      return {-1, 0.0};
    }

    // Log all visible tags
    for (const auto& [tag_id, dist] : visible_tags) {
      RCLCPP_INFO(get_logger(), "Tag %d: %.2f meters", tag_id, dist);
    }
    
    // Find nearest tag
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

/**
 * @brief Process AprilTag detection to estimate robot pose
 * 
 * @return Optional robot pose in map frame. Empty if pose cannot be determined
 */
std::optional<geometry_msgs::msg::Pose> SwitchMapNode::process_detection()
{
  try {
    // Find nearest visible tag
    auto [nearest_tag_id, distance] = find_visible_tags();
    if (nearest_tag_id < 0) {
      RCLCPP_WARN(get_logger(), "No AprilTags visible");
      return std::nullopt;
    }
    
    // Check if tag exists in current area info
    auto current_tags = all_tag_poses_[current_area_];
    if (current_tags.find(nearest_tag_id) == current_tags.end()) {
      RCLCPP_WARN(get_logger(), "Tag %d not found in current area (%s) info",
                  nearest_tag_id, current_area_.c_str());
      return std::nullopt;
    }

    // Get necessary transforms
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
    
    // Calculate robot pose in map frame
    auto robot_pose_map = multiply_transforms(
      map_to_tag.transform,
      invert_transform(base_to_tag.transform)
    );
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = robot_pose_map.transform.translation.x;
    pose.position.y = robot_pose_map.transform.translation.y;
    pose.position.z = robot_pose_map.transform.translation.z;
    pose.orientation = robot_pose_map.transform.rotation;
    
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

/**
 * @brief Helper function to invert a transform
 */
geometry_msgs::msg::Transform SwitchMapNode::invert_transform(
    const geometry_msgs::msg::Transform& transform)
{
  Eigen::Matrix4d mat = transform_to_matrix(transform);
  Eigen::Matrix4d inv_mat = mat.inverse();
  return matrix_to_transform(inv_mat);
}

/**
 * @brief Convert transform to 4x4 matrix
 */
Eigen::Matrix4d SwitchMapNode::transform_to_matrix(
    const geometry_msgs::msg::Transform& transform)
{
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  
  // Set rotation
  Eigen::Quaterniond q(
    transform.rotation.w,
    transform.rotation.x,
    transform.rotation.y,
    transform.rotation.z
  );
  mat.block<3,3>(0,0) = q.toRotationMatrix();
  
  // Set translation
  mat(0,3) = transform.translation.x;
  mat(1,3) = transform.translation.y;
  mat(2,3) = transform.translation.z;
  
  return mat;
}

/**
 * @brief Convert 4x4 matrix to transform
 */
geometry_msgs::msg::Transform SwitchMapNode::matrix_to_transform(
    const Eigen::Matrix4d& matrix)
{
  geometry_msgs::msg::Transform transform;
  
  // Extract translation
  transform.translation.x = matrix(0,3);
  transform.translation.y = matrix(1,3);
  transform.translation.z = matrix(2,3);
  
  // Extract rotation and convert to quaternion
  Eigen::Matrix3d rot = matrix.block<3,3>(0,0);
  Eigen::Quaterniond q(rot);
  transform.rotation.w = q.w();
  transform.rotation.x = q.x();
  transform.rotation.y = q.y();
  transform.rotation.z = q.z();
  
  return transform;
}

/**
 * @brief Multiply two transforms
 */
geometry_msgs::msg::TransformStamped SwitchMapNode::multiply_transforms(
    const geometry_msgs::msg::Transform& t1,
    const geometry_msgs::msg::Transform& t2)
{
  Eigen::Matrix4d m1 = transform_to_matrix(t1);
  Eigen::Matrix4d m2 = transform_to_matrix(t2);
  Eigen::Matrix4d result = m1 * m2;
  
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform = matrix_to_transform(result);
  return transform_stamped;
}