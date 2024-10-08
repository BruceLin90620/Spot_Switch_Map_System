#include <map_manager/map_manager.hpp>

MapManager::MapManager()
: Node("map_manager")
{
    initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    switch_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_map");

    current_map_id_ = 0;

    RCLCPP_INFO(this->get_logger(), "Map Navigation Node has been initialized.");
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
        }

        send_goal(goal_id);
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
            send_initial_pose();
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
    sleep(3);
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

void MapManager::send_initial_pose()
{
    auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    initial_pose_msg.header.frame_id = "map";
    initial_pose_msg.header.stamp = this->now();

    initial_pose_msg.pose.pose.position.x = 5.0;
    initial_pose_msg.pose.pose.position.y = 0.0;
    initial_pose_msg.pose.pose.orientation.w = 1.0;
    initial_pose_msg.pose.pose.orientation.x = 0.0;
    initial_pose_msg.pose.pose.orientation.y = 0.0;
    initial_pose_msg.pose.pose.orientation.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Sending initial pose");
    
    initial_pose_publisher_->publish(initial_pose_msg);
}