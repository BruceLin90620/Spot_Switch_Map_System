#include <map_manager/map_manager.hpp>


MapManager::MapManager(const std::string& config_file_path)
: Node("map_manager"), config_file_path_(config_file_path)
{
    initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
    
    switch_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_map");
    switch_spot_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_spot_map");

    auto qos_profile = rmw_qos_profile_services_default;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    
    send_goal_pose_client_ = this->create_client<switch_map_interfaces::srv::SendGoalPose>("send_goal_pose", qos_profile);

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
            switch_map(map_id, goal_id);
        }
        else
        {
            send_goal(goal_id);
        }
    }
}


void MapManager::switch_map(int new_map_id, int goal_id)
{
    auto request = std::make_shared<switch_map_interfaces::srv::SingleMap::Request>();
    request->mapid = new_map_id;

    RCLCPP_INFO(this->get_logger(), "Requesting to switch map to ID %d", new_map_id);

    auto spot_result_future = switch_spot_map_client_->async_send_request(request);
    auto result_future = switch_map_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        if (result->success)
        {
            current_map_id_ = new_map_id;
            RCLCPP_INFO(this->get_logger(), "Map switched to ID %d", new_map_id);
            send_initial_pose(goal_id);
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

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), spot_result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = spot_result_future.get();
        if (result->success)
        {
            RCLCPP_INFO(this->get_logger(), "Spot map switched to ID %d", new_map_id);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch spot map: Service returned failure");
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service call to switch spot map timed out");
    }
}


void MapManager::send_goal(int goal_id)
{
    RCLCPP_INFO(this->get_logger(), "Going to the target point....");
    auto request = std::make_shared<switch_map_interfaces::srv::SendGoalPose::Request>();

    request->goal_pose.header.frame_id = "map";
    request->goal_pose.header.stamp = this->now();

    try {

        const auto& coordinates = config_["entry_points"][current_map_id_]["points"][goal_id]["coordinates"];
        
        if (coordinates && coordinates.IsSequence() && coordinates.size() == 7) {

            request->goal_pose.pose.position.x = coordinates[0].as<double>();
            request->goal_pose.pose.position.y = coordinates[1].as<double>();
            request->goal_pose.pose.position.z = coordinates[2].as<double>();
            request->goal_pose.pose.orientation.x = coordinates[3].as<double>();
            request->goal_pose.pose.orientation.y = coordinates[4].as<double>();
            request->goal_pose.pose.orientation.z = coordinates[5].as<double>();
            request->goal_pose.pose.orientation.w = coordinates[6].as<double>();

            RCLCPP_INFO(this->get_logger(), "Sending goal pose for map %d, point %d", current_map_id_, goal_id);
            
            auto result_future = send_goal_pose_client_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = result_future.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Goal sent successfully");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service call to send goal failed");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid or missing coordinates for map %d, point %d", current_map_id_, goal_id);
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error accessing YAML config: %s", e.what());
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