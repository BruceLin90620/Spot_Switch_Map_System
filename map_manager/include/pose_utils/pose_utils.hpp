#ifndef POSE_UTILS_HPP
#define POSE_UTILS_HPP

#include <vector>
#include <utility>
#include <cmath>
#include <stdexcept>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace pose_utils {

struct Point {
    double x;
    double y;
};

struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

// 計算兩點之間的yaw角度
double calculateYaw(const Point& from, const Point& to);

// 將yaw角度轉換為四元數
Quaternion yawToQuaternion(double yaw);

// 找出相鄰的兩個點
std::pair<Point, Point> findAdjacentPoints(
    const std::vector<std::pair<int, Point>>& points, 
    int currentId);

// 計算pose的四元數
Quaternion calculatePoseQuaternion(
    const std::vector<std::pair<int, Point>>& points, 
    int targetPointId);

// 更新目標姿態
void updateGoalPose(
    geometry_msgs::msg::PoseStamped& goal_pose,
    const std::vector<std::pair<int, Point>>& points,
    int goal_id);

} // namespace pose_utils

#endif // POSE_UTILS_HPP