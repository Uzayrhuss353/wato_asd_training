#include "control_node.hpp"

ControlNode::ControlNode() : Node("control_node")
{
    // Parameters
    lookahead_distance_ = 1.0;  // meters
    goal_tolerance_ = 0.1;      // meters
    linear_speed_ = 0.5;        // m/s

    // Subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop()
{
    if (!current_path_ || !robot_odom_ || current_path_->poses.empty())
        return;

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point)
    {
        // Stop robot if no valid point
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        return;
    }

    geometry_msgs::msg::Twist cmd_vel = computeVelocity(*lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint()
{
    const auto &pose = robot_odom_->pose.pose;
    geometry_msgs::msg::Point robot_pos = pose.position;

    for (const auto &wp : current_path_->poses)
    {
        if (computeDistance(robot_pos, wp.pose.position) >= lookahead_distance_)
        {
            return wp;
        }
    }

    return {}; // No lookahead point found
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target)
{
    geometry_msgs::msg::Twist cmd_vel;

    const auto &pose = robot_odom_->pose.pose;
    double yaw = extractYaw(pose.orientation);

    double dx = target.pose.position.x - pose.position.x;
    double dy = target.pose.position.y - pose.position.y;

    double target_yaw = std::atan2(dy, dx);
    double angle_diff = target_yaw - yaw;

    // Normalize angle between -pi and pi
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2.0 * angle_diff; // gain for steering

    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat)
{
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
