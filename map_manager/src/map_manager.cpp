#include "map_manager/map_manager.hpp"

MapManager::MapManager()
: Node("map_manager"){

    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
    switch_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_map");
    
    // Configure QoS profile for reliable communication
    auto qos_profile = rmw_qos_profile_services_default;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    
    switch_spot_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_spot_map", qos_profile);
    send_goal_pose_client_ = this->create_client<switch_map_interfaces::srv::SendGoalPose>("/send_goal_pose", qos_profile);
    nav_service_client_ = this->create_client<routing_agent_interfaces::srv::NavServiceMsg>("/NavService", qos_profile);

    take_lease_client_ = this->create_client<std_srvs::srv::Trigger>("/take_lease_");

    RCLCPP_INFO(this->get_logger(), "Map Navigation Node has been initialized.");
}

/**
 * @brief Request and process navigation path from current position
 * @param current_position Current position identifier
 */
void MapManager::request_and_process_path(std::string& current_position) 
{
    // Prepare and send navigation service request
    auto request = std::make_shared<routing_agent_interfaces::srv::NavServiceMsg::Request>();
    request->can_arrive = true;
    request->i_am_at = current_position;
    current_position_ = current_position;
    
    RCLCPP_INFO(this->get_logger(), "Sending request to NavService from position: %s", 
                current_position.c_str());

    auto result_future = nav_service_client_->async_send_request(request);
    
    // Process navigation service response
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        process_navigation_response(result_future.get(), current_position);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call NavService");
        path_sequence_.clear();
    }
}

/**
 * @brief Process the navigation service response
 * @param result Navigation service response
 * @param current_position Current position identifier
 */
void MapManager::process_navigation_response(
    const routing_agent_interfaces::srv::NavServiceMsg::Response::SharedPtr& result, std::string& current_position){

    try {
        auto json_response = nlohmann::json::parse(result->path_to_next_task);
        path_sequence_ = json_response["node_sequence:"].get<std::vector<std::string>>();
        
        if (path_sequence_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No tasks available at the moment.");
            return;
        }

        // Update graph information
        update_graph_information(json_response["graph"]);
        
        // Log path sequence
        log_path_sequence();
        
        // Process first point in sequence
        if (!path_sequence_.empty()) {
            process_next_point(current_position);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing response: %s", e.what());
        path_sequence_.clear();
    }
}

/**
 * @brief Update the current graph with new information
 * @param new_graph New graph information in JSON format
 */
void MapManager::update_graph_information(const nlohmann::json& new_graph) {

    for (auto it = new_graph.begin(); it != new_graph.end(); ++it) {
        current_graph_[it.key()] = it.value();
    }
}

/**
 * @brief Log the complete path sequence
 */
void MapManager::log_path_sequence() {

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
}

/**
 * @brief Process the next point in the sequence
 * @param current_position Current position identifier
 */
void MapManager::process_next_point(std::string& current_position) 
{
    current_sequence_index_ = 0;
    const std::string& next_node_id = path_sequence_[0];
    
    // Extract map ID from node ID
    int map_id = std::stoi(next_node_id.substr(0, 3));
    
    if (map_id != current_map_id_) {
        switch_map(map_id, next_node_id);
    } else {
        send_goal(next_node_id, current_graph_[next_node_id], current_position);
    }
    
    current_position = next_node_id;
}

/**
 * @brief Switch to a new map
 * @param new_map_id ID of the new map
 * @param node_id Target node ID
 */
void MapManager::switch_map(int new_map_id, [[maybe_unused]] const std::string& node_id) {
    auto request = std::make_shared<switch_map_interfaces::srv::SingleMap::Request>();
    request->mapid = new_map_id;

    RCLCPP_INFO(this->get_logger(), "Requesting to switch map to ID %d", new_map_id);

    // Store the shared futures
    auto result_future = switch_map_client_->async_send_request(request);
    auto spot_result_future = switch_spot_map_client_->async_send_request(request);

    // rclcpp::sleep_for(std::chrono::seconds(2));
    // RCLCPP_INFO(this->get_logger(), "Wait for 2 sec");

    // Pass the shared futures by value
    bool success = process_map_switch_response( result_future.future.share(),
                                                spot_result_future.future.share(),
                                                new_map_id);

    if (success) {
        current_map_id_ = new_map_id;
        // send_goal(node_id, current_graph_[node_id]);
    }
}

/**
 * @brief Process the response from map switching services
 * @return true if both map switches were successful
 */
bool MapManager::process_map_switch_response(
    std::shared_future<switch_map_interfaces::srv::SingleMap::Response::SharedPtr> result_future,
    std::shared_future<switch_map_interfaces::srv::SingleMap::Response::SharedPtr> spot_result_future,
    int new_map_id) {
    
    bool success = true;

    // Process main map switch response
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

    // Process spot map switch response
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

    return success;
}

/**
 * @brief Send a goal position to the navigation stack
 * @param node_id Target node ID
 * @param node_info Node information from the graph
 * @param current_position Current position identifier
 */
void MapManager::send_goal(
    const std::string& node_id, 
    const nlohmann::json& node_info, 
    const std::string& current_position) 
{
    auto request = std::make_shared<switch_map_interfaces::srv::SendGoalPose::Request>();
    request->goal_pose.header.frame_id = "map";
    request->goal_pose.header.stamp = this->now();

    try {
        prepare_goal_pose(request, node_info, current_position, node_id);

        auto result_future = send_goal_pose_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            if (!result->success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
                handle_send_failure();
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal sent successfully");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call to send goal failed");
            handle_send_failure();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing goal: %s", e.what());
    }
}

/**
 * @brief Prepare the goal pose message
 */
void MapManager::prepare_goal_pose(
    switch_map_interfaces::srv::SendGoalPose::Request::SharedPtr& request,
    const nlohmann::json& node_info,
    const std::string& current_position,
    const std::string& node_id){
    
    // Get target point coordinates
    const auto& target_location = node_info["local_location"];
    double target_x = target_location[0].get<double>();
    double target_y = target_location[1].get<double>();

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

    // Set position
    request->goal_pose.pose.position.x = target_x;
    request->goal_pose.pose.position.y = target_y;
    request->goal_pose.pose.position.z = target_location[2].get<double>();

    // Calculate yaw angle from current position to target
    double dx = target_x - current_x;
    double dy = target_y - current_y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // Roll and Pitch are 0, only set Yaw
    q.normalize();

    // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
    request->goal_pose.pose.orientation.x = q.x();
    request->goal_pose.pose.orientation.y = q.y();
    request->goal_pose.pose.orientation.z = q.z();
    request->goal_pose.pose.orientation.w = q.w();

    RCLCPP_INFO(this->get_logger(), "Preparing goal from %s to %s:", 
                current_position.c_str(), node_id.c_str());
    RCLCPP_INFO(this->get_logger(), "From: (%.2f, %.2f) To: (%.2f, %.2f)",
                current_x, current_y, target_x, target_y);
}

/**
 * @brief Handle failure in sending goal
 */
void MapManager::handle_send_failure() 
{
    std::cout << "\nGoal sending failed. Please choose an action:\n"
              << "1: Retry current goal\n"
              << "2: Abort\n"
              << "Enter your choice (1-2): ";

    std::string input;
    if(!std::getline(std::cin, input) || input.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid input, aborting");
        path_sequence_.clear();
        return;
    }

    switch (input[0]) {
        case '1':
            RCLCPP_INFO(this->get_logger(), "Processing retry request...");

            if (!request_lease()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to acquire lease");
                break;
            }
        
            RCLCPP_INFO(this->get_logger(), "Retrying current goal...");
            send_goal(path_sequence_[0], 
                     current_graph_[path_sequence_[0]],
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

/**
 * @brief Request a service to take lease
 */
bool MapManager::request_lease() {

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = take_lease_client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Sending take lease request...");

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) 
    {
        auto result = result_future.get();
        if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Lease taken successfully: %s", result->message.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to take lease: %s", result->message.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call lease service");
        return false;
    }
}