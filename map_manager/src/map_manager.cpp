#include "map_manager/map_manager.hpp"

MapManager::MapManager()
: Node("map_manager")
{
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
    
    switch_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_map");
    switch_spot_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_spot_map");
    
    auto qos_profile = rmw_qos_profile_services_default;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    
    send_goal_pose_client_ = this->create_client<switch_map_interfaces::srv::SendGoalPose>(
        "send_goal_pose", qos_profile);
    
    nav_service_client_ = this->create_client<routing_agent_interfaces::srv::NavServiceMsg>("/NavService", qos_profile);

    RCLCPP_INFO(this->get_logger(), "Map Navigation Node has been initialized.");
}

void MapManager::request_and_process_path(std::string& current_position) {
    auto request = std::make_shared<routing_agent_interfaces::srv::NavServiceMsg::Request>();
    request->can_arrive = true;
    request->i_am_at = current_position;
    current_position_ = current_position;
    RCLCPP_INFO(this->get_logger(), "Sending request to NavService from position: %s", current_position.c_str());

    auto result_future = nav_service_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        try {
            auto json_response = nlohmann::json::parse(result->path_to_next_task);
            path_sequence_ = json_response["node_sequence:"].get<std::vector<std::string>>();
            
            if (path_sequence_.empty()) {
                RCLCPP_INFO(this->get_logger(), "No tasks available at the moment.");
                return;
            }

            // Merge new graph information instead of overwriting
            auto new_graph = json_response["graph"];
            for (auto it = new_graph.begin(); it != new_graph.end(); ++it) {
                current_graph_[it.key()] = it.value();
            }
            
            // Log the complete path sequence but only process first point
            RCLCPP_INFO(this->get_logger(), "Received path sequence with %zu points:", path_sequence_.size());
            for (size_t i = 0; i < path_sequence_.size(); ++i) {
                const std::string& node_id = path_sequence_[i];
                const auto& location = current_graph_[node_id]["local_location"];
                RCLCPP_INFO(this->get_logger(), "Point %zu: %s (%.2f, %.2f)",
                    i + 1,
                    node_id.c_str(),
                    location[0].get<double>(),
                    location[1].get<double>()
                );
            }
            
            // Process the first point
            if (!path_sequence_.empty()) {
                current_sequence_index_ = 0;
                const std::string& next_node_id = path_sequence_[0];
                
                // Extract map ID from node ID
                int map_id = std::stoi(next_node_id.substr(0, 3));
                
                if (map_id != current_map_id_) {
                    switch_map(map_id, next_node_id);
                } else {
                    // Use current position to send goal
                    send_goal(next_node_id, current_graph_[next_node_id], current_position);
                }
                
                // Update current position after sending goal
                current_position = next_node_id;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing response: %s", e.what());
            path_sequence_.clear();
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call NavService");
        path_sequence_.clear();
    }
}


void MapManager::switch_map(int new_map_id, const std::string& node_id) {
    auto request = std::make_shared<switch_map_interfaces::srv::SingleMap::Request>();
    request->mapid = new_map_id;

    RCLCPP_INFO(this->get_logger(), "Requesting to switch map to ID %d", new_map_id);

    auto result_future = switch_map_client_->async_send_request(request);
    auto spot_result_future = switch_spot_map_client_->async_send_request(request);

    bool success = true;

    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "Wait for 2 sec");

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
        rclcpp::FutureReturnCode::SUCCESS) 
    {
        auto result = result_future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Map switched to ID %d", new_map_id);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch map");
            success = false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Service call to switch map timed out");
        success = false;
    }

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), spot_result_future) == 
        rclcpp::FutureReturnCode::SUCCESS) 
    {
        auto result = spot_result_future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Spot map switched to ID %d", new_map_id);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch spot map");
            success = false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Service call to switch spot map timed out");
        success = false;
    }

    if (success) {
        current_map_id_ = new_map_id;
        // send_goal(node_id, current_graph_[node_id]);
    }
}

std::string MapManager::get_next_position() const {
    if (!path_sequence_.empty() && current_sequence_index_ < path_sequence_.size()) {
        return path_sequence_[current_sequence_index_];
    }
    return "";
}

void MapManager::send_goal(const std::string& node_id, const nlohmann::json& node_info, const std::string& current_position) {
    auto request = std::make_shared<switch_map_interfaces::srv::SendGoalPose::Request>();
    request->goal_pose.header.frame_id = "map";
    request->goal_pose.header.stamp = this->now();

    try {
        // Get target point coordinates from node_info
        const auto& target_location = node_info["local_location"];
        pose_utils::Point targetPoint{
            target_location[0].get<double>(),
            target_location[1].get<double>()
        };

        // Get current position coordinates
        double current_x = 0.0, current_y = 0.0;
        if (current_graph_.contains(current_position)) {
            const auto& current_location = current_graph_[current_position]["local_location"];
            current_x = current_location[0].get<double>();
            current_y = current_location[1].get<double>();
        } else {
            RCLCPP_WARN(this->get_logger(), "Current position %s not found in graph, using (0,0)", 
                        current_position.c_str());
        }
        pose_utils::Point currentPoint{current_x, current_y};

        // Set position
        request->goal_pose.pose.position.x = targetPoint.x;
        request->goal_pose.pose.position.y = targetPoint.y;
        request->goal_pose.pose.position.z = target_location[2].get<double>();

        // Calculate orientation based on current position to target
        double yaw = pose_utils::calculateYaw(currentPoint, targetPoint);
        auto q = pose_utils::yawToQuaternion(yaw);
        
        request->goal_pose.pose.orientation.x = q.x;
        request->goal_pose.pose.orientation.y = q.y;
        request->goal_pose.pose.orientation.z = q.z;
        request->goal_pose.pose.orientation.w = q.w;

        // Log correct start and end positions
        RCLCPP_INFO(this->get_logger(), "Preparing goal from %s to %s:", 
                    current_position.c_str(), node_id.c_str());
        RCLCPP_INFO(this->get_logger(), "From: (%.2f, %.2f) To: (%.2f, %.2f)",
                    current_x, current_y, targetPoint.x, targetPoint.y);

        // rclcpp::sleep_for(std::chrono::seconds(2));
        // Uncomment the following code when ready to send actual goals
        auto result_future = send_goal_pose_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Goal sent successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
                handle_send_failure();
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call to send goal failed");
            handle_send_failure();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing goal: %s", e.what());
    }
}

void MapManager::handle_send_failure() {
    std::cout << "\nGoal sending failed. Please choose an action:\n";
    std::cout << "1: Retry current goal\n";
    std::cout << "2: Abort\n";
    std::cout << "Enter your choice (1-2): ";

    std::string input;
    if(!std::getline(std::cin, input) || input.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid input, aborting");
        path_sequence_.clear();
        return;
    }

    switch (input[0]) {
        case '1':
            RCLCPP_INFO(this->get_logger(), "Retrying current goal...");
            send_goal(path_sequence_[current_sequence_index_], 
                     current_graph_[path_sequence_[current_sequence_index_]],
                     current_position_);
            break;
        case '2':
            RCLCPP_INFO(this->get_logger(), "Aborting navigation...");
            path_sequence_.clear();
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid input. Please try again.");
            handle_send_failure();
            break;
    }
}