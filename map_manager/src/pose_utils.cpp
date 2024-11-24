// pose_utils.cpp
#include "pose_utils/pose_utils.hpp"
#include <rclcpp/rclcpp.hpp>

namespace pose_utils {

double calculateYaw(const Point& from, const Point& to) {
    double yaw = atan2(to.y - from.y, to.x - from.x);
    
    if (yaw > M_PI) {
        yaw -= 2 * M_PI;
    } else if (yaw < -M_PI) {
        yaw += 2 * M_PI;
    }
    
    return yaw;
}

Quaternion yawToQuaternion(double yaw) {
    return {
        0.0,                    // x
        0.0,                    // y
        sin(yaw / 2.0),        // z
        cos(yaw / 2.0)         // w
    };
}


std::pair<Point, Point> findAdjacentPoints(
    const std::vector<std::pair<int, Point>>& points, 
    int currentId) 
{
    std::size_t currentIndex = 0;
    bool found = false;
    
    for (std::size_t i = 0; i < points.size(); i++) {
        if (points[i].first == currentId) {
            currentIndex = i;
            found = true;
            break;
        }
    }
    
    if (!found) {
        throw std::runtime_error("Current point ID not found");
    }
    
    Point currentPoint = points[currentIndex].second;
    Point referencePoint;
    bool isReversed = false;
    
    if (currentIndex > 0) {
        referencePoint = points[currentIndex - 1].second;
    }

    else if (currentIndex + 1 < points.size()) {
        referencePoint = points[currentIndex + 1].second;
        isReversed = true;
        RCLCPP_INFO(rclcpp::get_logger("pose_calculator"), "First point detected, reversing direction");
    }
    else {
        throw std::runtime_error("Cannot determine orientation - insufficient points");
    }
    
    if (isReversed) {
        return std::make_pair(currentPoint, referencePoint);
    } else {
        return std::make_pair(referencePoint, currentPoint);
    }
}



Quaternion calculatePoseQuaternion(
    const std::vector<std::pair<int, Point>>& points, 
    int targetPointId) 
{
    auto [from, to] = findAdjacentPoints(points, targetPointId);
    double yaw = calculateYaw(from, to);
    return yawToQuaternion(yaw);
}

void updateGoalPose(
    geometry_msgs::msg::PoseStamped& goal_pose,
    const std::vector<std::pair<int, Point>>& points,
    int goal_id)
{
    try {
        Quaternion q = calculatePoseQuaternion(points, goal_id);
        
        goal_pose.pose.orientation.x = q.x;
        goal_pose.pose.orientation.y = q.y;
        goal_pose.pose.orientation.z = q.z;
        goal_pose.pose.orientation.w = q.w;
        
        for (const auto& [id, point] : points) {
            if (id == goal_id) {
                goal_pose.pose.position.x = point.x;
                goal_pose.pose.position.y = point.y;
                goal_pose.pose.position.z = 0.0;
                break;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pose_calculator"), 
                     "Error calculating pose: %s", e.what());
        throw;
    }
}

} // namespace pose_utils