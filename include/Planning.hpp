#ifndef PLANNING_HPP
#define PLANNING_HPP

#include <vector>
#include <algorithm>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/get_plan.hpp"

// A-star cell structure
struct Cell {
    int x, y;
    float f, g, h;
    std::shared_ptr<Cell> parent;

    Cell(int c, int r);
};

bool operator<(const Cell &a, const Cell &b)
{
    return a.f < b.f;
}

bool operator<(const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>& b) 
{
    if(a->f < b->f)
        return true;
    else
        return false;
}

bool operator==(const Cell &a, const Cell &b)
{
    return (a.x == b.x) && (a.y == b.y);
}
bool operator==(const std::shared_ptr<Cell>& lhs, const std::shared_ptr<Cell>& rhs) 
{
    if (!lhs || !rhs) {
        return false; // Handle null pointers
    }
    return (lhs->x == rhs->x) && (lhs->y == rhs->y); // Compare the underlying Cell objects
}


class PlanningNode : public rclcpp::Node {
public:
    PlanningNode();

private:
    // Parameters
    // TO DO

    // Callbacks
    void mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future);
    void planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response);

    // Clients
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;

    // Services
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_service_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Methods
    void dilateMap();
    void aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal);
    void smoothPath();

    // Data
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Path path_;
};


#endif // PLANNING_HPP
