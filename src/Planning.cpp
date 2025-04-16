#include "Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");
        
        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>("plan_path",std::bind(&PlanningNode::planPath,this,std::placeholders::_1,std::placeholders::_2));
        
        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path",1);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while (!map_client_->wait_for_service(std::chrono::seconds(5)) && rclcpp::ok);
        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto request_map = map_client_->async_send_request(request,std::bind(&PlanningNode::mapCallback,this,std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {

    auto response = future.get();
    if (response)
    {
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "I GOT A JAR OF MAP!");
    }
    
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
{
    // add code here

    // ********
    // * Help *
    // ********
    
    aStar(request->start, request->goal);
    smoothPath();

    response->set__plan(path_);
    path_pub_->publish(path_);
}

void PlanningNode::dilateMap() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    // add code here

    // ********
    // * Help *
    // ********
    
    Cell cStart(start.pose.position.x,start.pose.position.y);
    Cell cGoal(goal.pose.position.x,goal.pose.position.y);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    int dirX[] = {1, 0, -1, 0};
    int dirY[] = {0, 1, 0, -1};

    while(!openList.empty() && rclcpp::ok())
    {
        std::sort(openList.begin(),openList.end());
        auto current = openList.front();

        if(current->x == cGoal.x && current->y == cGoal.y)
            break;
        
        for (int i = 0; i<4; i++) //vsetky mozne cesty
        {
            int xNew = current->x + dirX[i];
            int yNew = current->y + dirY[i];
            Cell NewCell(xNew, yNew);

            if (xNew > 0 && xNew < map_.info.width && yNew > 0 && yNew < map_.info.width)
            {
                if (map_.data[yNew*map_.info.width + xNew] > 75 && closedList[yNew*map_.info.width + xNew] == false)
                {
                    auto it = std::find(openList.begin(),openList.end(),std::make_shared<Cell>(NewCell));
                    if (openList.end() != it)
                    {
                        it->get()->g = current->g + 1;
                        it->get()->f = it->get()->g + it->get()->h;
                    }
                    else
                    {
                        NewCell.g = current->g + 1;
                        NewCell.h = std::sqrt((current->x - NewCell.x)^2 + (current->y - NewCell.y)^2);
                        NewCell.f = NewCell.g + NewCell.h;
                        openList.push_back(std::make_shared<Cell>(NewCell));
                    }
                    
                }
            }
            
        }

    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    
}

void PlanningNode::smoothPath() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
}

Cell::Cell(int c, int r) {
    // add code here
}
