#include "map_manager/map_manager.hpp"

MapManager::MapManager(const std::string& config_file_path)
: Node("map_manager"), config_file_path_(config_file_path)
{
    switch_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_map");
    switch_spot_map_client_ = this->create_client<switch_map_interfaces::srv::SingleMap>("/switch_spot_map");

    auto qos_profile = rmw_qos_profile_services_default;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    
    send_goal_pose_client_ = this->create_client<switch_map_interfaces::srv::SendGoalPose>(
        "send_goal_pose", qos_profile);

    routing_path_service_ = this->create_service<switch_map_interfaces::srv::GoalPath>(
        "goal_path",
        std::bind(&MapManager::MapManager, this, 
                  std::placeholders::_1, std::placeholders::_2)
    );

    load_config();
    RCLCPP_INFO(this->get_logger(), "Map Navigation Node has been initialized.");
}

void MapManager::load_config() {
    try {
        config_ = YAML::LoadFile(config_file_path_);
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading YAML file: %s", e.what());
        throw;
    }
}

void MapManager::process_input(const std::vector<std::string>& input_vector) {
    path_sequence_.clear();
    
    for (const auto& input : input_vector) {
        if (input.length() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Invalid input format: %s", input.c_str());
            continue;
        }

        try {
            int map_id = std::stoi(input.substr(0, 1));
            int goal_id = std::stoi(input.substr(1));
            path_sequence_.emplace_back(map_id, goal_id);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing input '%s': %s", 
                        input.c_str(), e.what());
        }
    }
    
    for (size_t i = 0; i < path_sequence_.size(); ++i) {
        current_sequence_index_ = i;
        auto [map_id, goal_id] = path_sequence_[i];
        
        if (map_id != current_map_id_) {
            switch_map(map_id, goal_id);
        } else {
            send_goal(goal_id);
        }
    }
}

std::pair<int, int> MapManager::getNextPoint(size_t current_index) const {
    if (current_index + 1 < path_sequence_.size()) {
        return path_sequence_[current_index + 1];
    }
    return {-1, -1};
}

std::pair<int, int> MapManager::getPreviousPoint(size_t current_index) const {
    if (current_index > 0) {
        return path_sequence_[current_index - 1];
    }
    return {-1, -1};
}

void MapManager::switch_map(int new_map_id, int goal_id) {
    auto request = std::make_shared<switch_map_interfaces::srv::SingleMap::Request>();
    request->mapid = new_map_id;

    RCLCPP_INFO(this->get_logger(), "Requesting to switch map to ID %d", new_map_id);

    auto result_future = switch_map_client_->async_send_request(request);
    auto spot_result_future = switch_spot_map_client_->async_send_request(request);

    bool success = true;

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
        if (goal_id >= 0) {
            send_goal(goal_id);
        }
    }
}

void MapManager::send_goal(int goal_id) {
    auto request = std::make_shared<switch_map_interfaces::srv::SendGoalPose::Request>();
    request->goal_pose.header.frame_id = "map";
    request->goal_pose.header.stamp = this->now();

    try {
        std::vector<std::pair<int, pose_utils::Point>> points;
        
        const auto& map_points = config_["entry_points"][current_map_id_]["points"];
        for (const auto& point : map_points) {
            int point_id = point["point_id"].as<int>();
            const auto& coords = point["coordinates"];
            
            if (coords && coords.IsSequence() && coords.size() == 2) {
                points.push_back({
                    point_id,
                    pose_utils::Point{coords[0].as<double>(), coords[1].as<double>()}
                });
            }
        }

        if (points.size() >= 1) {
            auto [prev_map_id, prev_point_id] = getPreviousPoint(current_sequence_index_);
            auto [next_map_id, next_point_id] = getNextPoint(current_sequence_index_);

            pose_utils::Point currentPoint;
            pose_utils::Point referencePoint;
            bool foundCurrent = false;
            bool foundReference = false;

            for (const auto& [id, point] : points) {
                if (id == goal_id) {
                    currentPoint = point;
                    foundCurrent = true;
                    break;
                }
            }

            if (!foundCurrent) {
                throw std::runtime_error("Current point not found in map data");
            }

            if (prev_map_id == current_map_id_) {

                for (const auto& [id, point] : points) {
                    if (id == prev_point_id) {
                        referencePoint = point;
                        foundReference = true;

                        request->goal_pose.pose.position.x = currentPoint.x;
                        request->goal_pose.pose.position.y = currentPoint.y;
                        request->goal_pose.pose.position.z = 0.0;
                        double yaw = pose_utils::calculateYaw(referencePoint, currentPoint);
                        auto q = pose_utils::yawToQuaternion(yaw);
                        request->goal_pose.pose.orientation.x = q.x;
                        request->goal_pose.pose.orientation.y = q.y;
                        request->goal_pose.pose.orientation.z = q.z;
                        request->goal_pose.pose.orientation.w = q.w;
                        break;
                    }
                }
            } else if (next_map_id == current_map_id_) {
                for (const auto& [id, point] : points) {
                    if (id == next_point_id) {
                        referencePoint = point;
                        foundReference = true;
                        request->goal_pose.pose.position.x = currentPoint.x;
                        request->goal_pose.pose.position.y = currentPoint.y;
                        request->goal_pose.pose.position.z = 0.0;
                        double yaw = pose_utils::calculateYaw(currentPoint, referencePoint);
                        auto q = pose_utils::yawToQuaternion(yaw);
                        request->goal_pose.pose.orientation.x = q.x;
                        request->goal_pose.pose.orientation.y = q.y;
                        request->goal_pose.pose.orientation.z = q.z;
                        request->goal_pose.pose.orientation.w = q.w;
                        break;
                    }
                }
            }

            if (!foundReference) {

                request->goal_pose.pose.position.x = currentPoint.x;
                request->goal_pose.pose.position.y = currentPoint.y;
                request->goal_pose.pose.position.z = 0.0;
                request->goal_pose.pose.orientation.w = 1.0; 
                request->goal_pose.pose.orientation.x = 0.0;
                request->goal_pose.pose.orientation.y = 0.0;
                request->goal_pose.pose.orientation.z = 0.0;
            }

            std::cout << "\nPreparing to send goal:" << std::endl;
            std::cout << "Map ID: " << current_map_id_ << std::endl;
            std::cout << "Goal ID: " << goal_id << std::endl;
            std::cout << "Position: (" << request->goal_pose.pose.position.x 
                    << ", " << request->goal_pose.pose.position.y << ")" << std::endl;
            

            RCLCPP_INFO(this->get_logger(), "Sending goal as confirmed by user...");
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

        }
        else {
            RCLCPP_WARN(this->get_logger(), "No points found for map %d", current_map_id_);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing goal: %s", e.what());
    }
}


void MapManager::handle_send_failure() {
    std::cout << "\nGoal sending failed. Please choose an action:\n";
    std::cout << "1: Retry current goal\n";
    std::cout << "2: Skip to next goal\n";
    std::cout << "3: Abort sequence\n";
    std::cout << "Enter your choice (1-3): ";

    std::string input;
    if(!std::getline(std::cin, input) || input.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid input, aborting sequence");
        path_sequence_.clear();
        return;
    }

    switch (input[0]) {
        case '1':
            RCLCPP_INFO(this->get_logger(), "Retrying current goal...");
            send_goal(path_sequence_[current_sequence_index_].second);
            break;
        case '2':
            if (current_sequence_index_ < path_sequence_.size() - 1) {
                RCLCPP_INFO(this->get_logger(), "Skipping to next goal...");
                current_sequence_index_++;
            } else {
                RCLCPP_INFO(this->get_logger(), "No more goals in sequence. Aborting...");
                path_sequence_.clear();
            }
            break;
        case '3':
            RCLCPP_INFO(this->get_logger(), "Aborting goal sequence...");
            path_sequence_.clear();
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid input. Please try again.");
            handle_send_failure();
            break;
    }
}