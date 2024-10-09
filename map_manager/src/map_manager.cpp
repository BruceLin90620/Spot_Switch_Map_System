#include <map_manager/map_manager.hpp>

MapManager::MapManager(const std::string& config_file_path)
: Node("map_manager"), config_file_path_(config_file_path)
{
    initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    switch_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_map");

    load_config();

    current_map_id_ = 0;

    RCLCPP_INFO(this->get_logger(), "Map Navigation Node has been initialized.");
}

void MapManager::load_config()
{
    try {
        config_ = YAML::LoadFile(config_file_path_);
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading YAML file: %s", e.what());
    }
}

// void MapManager::load_config()
// {
//     try {
//         YAML::Node config = YAML::LoadFile(config_file_path_);
//         if (config["entry_points"]) {
//             config_ = config["entry_points"].as<std::vector<std::string>>();
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "No 'map_paths' key found in the configuration file");
//         }
//     } catch (const YAML::Exception& e) {
//         RCLCPP_ERROR(this->get_logger(), "Error reading YAML file: %s", e.what());
//     }
// }

void MapManager::process_input(const std::vector<std::string>& input_vector)
{
    for (const auto& input : input_vector)
    {
        int map_id = std::stoi(input.substr(0, 1));
        int goal_id = std::stoi(input.substr(1));

        if (map_id != current_map_id_)
        {
            switch_map(map_id);
            sleep(5);
            send_initial_pose(goal_id);
        }
        else
        {
            send_goal(goal_id);
        }

        
        
    }
}

void MapManager::switch_map(int new_map_id)
{
    auto request = std::make_shared<switch_map_interfaces::srv::SingleMap::Request>();
    request->mapid = new_map_id;

    RCLCPP_INFO(this->get_logger(), "Requesting to switch map to ID %d", new_map_id);

    auto result_future = switch_map_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        if (result->success)
        {
            current_map_id_ = new_map_id;
            RCLCPP_INFO(this->get_logger(), "Map switched to ID %d", new_map_id);
            // send_initial_pose(goal_id);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch map: Service returned failure");
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service call to switch map timed out");
    }
}

void MapManager::send_goal(int goal_id)
{
    RCLCPP_INFO(this->get_logger(), "Going to the target point....");
    sleep(20);
    // auto goal_msg = geometry_msgs::msg::PoseStamped();
    // goal_msg.header.frame_id = "map";
    // goal_msg.header.stamp = this->now();

    // goal_msg.pose.position.x = goal_id * 1.0;
    // goal_msg.pose.position.y = goal_id * 1.0;
    // goal_msg.pose.orientation.w = 1.0;

    // RCLCPP_INFO(this->get_logger(), "Sending goal to position (%f, %f)", 
    //             goal_msg.pose.position.x, goal_msg.pose.position.y);
    
    // goal_publisher_->publish(goal_msg);
}

void MapManager::send_initial_pose(int goal_id)
{
    auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    initial_pose_msg.header.frame_id = "map";
    initial_pose_msg.header.stamp = this->now();

    try {
        const auto& coordinates = config_["entry_points"][current_map_id_]["points"][goal_id]["coordinates"];
        
        if (coordinates && coordinates.IsSequence() && coordinates.size() == 7) {
            initial_pose_msg.pose.pose.position.x = coordinates[0].as<double>();
            initial_pose_msg.pose.pose.position.y = coordinates[1].as<double>();
            initial_pose_msg.pose.pose.position.z = coordinates[2].as<double>();
            initial_pose_msg.pose.pose.orientation.x = coordinates[3].as<double>();
            initial_pose_msg.pose.pose.orientation.y = coordinates[4].as<double>();
            initial_pose_msg.pose.pose.orientation.z = coordinates[5].as<double>();
            initial_pose_msg.pose.pose.orientation.w = coordinates[6].as<double>();

            RCLCPP_INFO(this->get_logger(), "Sending initial pose for map %d, point %d", current_map_id_, goal_id);
            initial_pose_publisher_->publish(initial_pose_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid or missing coordinates for map %d, point %d", current_map_id_, goal_id);
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error accessing YAML config: %s", e.what());
    }
}