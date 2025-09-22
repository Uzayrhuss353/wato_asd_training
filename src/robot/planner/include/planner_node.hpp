#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unordered_map>
#include <queue>
#include <vector>
#include <cmath>
#include "planner_core.hpp"  // wherever PlannerCore lives


// ------------------- Supporting Structures -------------------
struct CellIndex
{
    int x, y;
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
    bool operator==(const CellIndex &other) const { return x == other.x && y == other.y; }
};

struct CellIndexHash
{
    size_t operator()(const CellIndex &idx) const
    {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

struct AStarNode
{
    CellIndex index;
    double f_score;  // f = g + h
    double g_score;
    CellIndex parent;
    AStarNode(CellIndex idx, double f, double g, CellIndex p)
        : index(idx), f_score(f), g_score(g), parent(p) {}
};

struct CompareF
{
    bool operator()(const AStarNode &a, const AStarNode &b) { return a.f_score > b.f_score; }
};

// ------------------- PlannerNode -------------------
class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode();

private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
    bool goal_received_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    bool goalReached();
    void planPath();
    double heuristic(const CellIndex &a, const CellIndex &b);
    bool isOccupied(const CellIndex &cell);
};
