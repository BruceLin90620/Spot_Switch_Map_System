#include <map_manager/map_manager.hpp>

MapManager::MapManager(const std::string& config_file_path)
: Node("map_manager"), config_file_path_(config_file_path)
{
    initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
    switch_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_map");
    switch_spot_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_spot_map");


    send_goal_pose_action_client = rclcpp_action::create_client<SendGoalPose>(
        this,
        "send_goal_pose");

    load_config();

    current_map_id_ = 1;

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


void MapManager::process_input(const std::vector<std::string>& input_vector)
{
    for (const auto& input : input_vector)
    {
        int map_id = std::stoi(input.substr(0, 1));
        int goal_id = std::stoi(input.substr(1));

        if (map_id != current_map_id_)
        {
            switch_map(map_id);
            // sleep(5);
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

    auto spot_result_future = switch_spot_map_client_->async_send_request(request);
    auto result_future = switch_map_client_->async_send_request(request);

    if ((rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS) &&
        (rclcpp::spin_until_future_complete(this->get_node_base_interface(), spot_result_future) == rclcpp::FutureReturnCode::SUCCESS))
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

    auto goal_msg = switch_map_interfaces::action::SendGoalPose::Goal();
    // auto send_goal_options = rclcpp_action::Client<switch_map_interfaces::action::SendGoalPose>::SendGoalOptions();
    // send_goal_options.goal_response_callback = custom_qos
    goal_msg.goal_pose.header.frame_id = "map";
    goal_msg.goal_pose.header.stamp = this->now();

    try {
        const auto& coordinates = config_["entry_points"][current_map_id_]["points"][goal_id]["coordinates"];
        
        if (coordinates && coordinates.IsSequence() && coordinates.size() == 7) {
            goal_msg.goal_pose.pose.position.x = coordinates[0].as<double>();
            goal_msg.goal_pose.pose.position.y = coordinates[1].as<double>();
            goal_msg.goal_pose.pose.position.z = coordinates[2].as<double>();
            goal_msg.goal_pose.pose.orientation.x = coordinates[3].as<double>();
            goal_msg.goal_pose.pose.orientation.y = coordinates[4].as<double>();
            goal_msg.goal_pose.pose.orientation.z = coordinates[5].as<double>();
            goal_msg.goal_pose.pose.orientation.w = coordinates[6].as<double>();

            RCLCPP_INFO(this->get_logger(), "Sending goal pose for map %d, point %d", current_map_id_, goal_id);
            
            auto send_goal_options = rclcpp_action::Client<switch_map_interfaces::action::SendGoalPose>::SendGoalOptions();
            // send_goal_options.goal_response_callback =
            //     std::bind(&MapManager::goal_response_callback, this, std::placeholders::_1);
            // send_goal_options.feedback_callback =
            //     std::bind(&MapManager::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&MapManager::result_callback, this, std::placeholders::_1);

            auto goal_handle_future = send_goal_pose_action_client->async_send_goal(goal_msg, send_goal_options);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
                return;
            }

            auto goal_handle = goal_handle_future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                return;
            }

            auto result_future = send_goal_pose_action_client->async_get_result(goal_handle);

            RCLCPP_INFO(this->get_logger(), "Waiting for the action to complete...");
            
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get result");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Action completed");

        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid or missing coordinates for map %d, point %d", current_map_id_, goal_id);
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error accessing YAML config: %s", e.what());
    }
}

void MapManager::result_callback(const GoalHandleSendGoalPose::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
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