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
        while (!map_client_->wait_for_service(std::chrono::seconds(5)))
        {
            if(rclcpp::ok() == false)
                break;
        }
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

    float resolution = map_.info.resolution;
    float origin_x = map_.info.origin.position.x;
    float origin_y = map_.info.origin.position.y;
    
    Cell cStart((start.pose.position.x+7.1)/resolution,(start.pose.position.y+3.6)/resolution);
    Cell cGoal((goal.pose.position.x+7.1)/resolution,(goal.pose.position.y+3.6)/resolution);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    


    openList.push_back(std::make_shared<Cell>(cStart));

    std::shared_ptr<Cell> goalCell = nullptr;

    //int dirX[] = {1, 1, 0, -1, -1, -1, 0, 1};
    //int dirY[] = {0, 1, 1,  1,  0, -1,-1,-1};
    int dirX[] = {1,0,-1,0};
    int dirY[] = {0,1,0,-1};

    while(!openList.empty())
    {
        if (rclcpp::ok() == false)
        {
            break;
        }
        
        std::sort(openList.begin(),openList.end());
        auto current = openList.front();

        if(current->x == cGoal.x && current->y == cGoal.y)
        {
            RCLCPP_INFO(get_logger(),"Goal Reached");
            goalCell = current;
            break;
        }
        
        
        for (int i = 0; i<4; i++) //vsetky mozne cesty
        {
            uint32_t xNew = current->x + dirX[i];
            uint32_t yNew = current->y + dirY[i];
            Cell NewCell(xNew,yNew);
            NewCell.x = xNew;
            NewCell.y = yNew;

            if (xNew < map_.info.width && yNew < map_.info.width)
            {
                //RCLCPP_INFO(get_logger(),"Inside of map!");
                if (map_.data[yNew*map_.info.width + xNew] < 50 && closedList[yNew*map_.info.width + xNew] == false)
                {
                    //RCLCPP_INFO(get_logger(),"Nepozeram na stenu!");
                    auto it = std::find(openList.begin(),openList.end(),std::make_shared<Cell>(NewCell));
                    /*
                    for(auto i = 0; i < openList.size(); ++i)
                    {
                        uint32_t xVar = openList.at(i); //TODO: fetch the data
                        uint32_t yVar = openList.at(i); //TODO: fetch the data

                        if(xVar == xNew && yVar == yNew)
                        {
                            
                        }
                    }
                    */
                    if (openList.end() != it)
                    {
                        if(it->get()->parent.get()->g > current.get()->g)
                        {
                            it->get()->g = current->g + 1;
                            it->get()->f = it->get()->g + it->get()->h;
                            it->get()->parent = current;
                        }
                        
                        //RCLCPP_INFO(get_logger(),"Old visited!");
                    }
                    else
                    {
                        NewCell.g = current->g + 1;
                        NewCell.h = std::sqrt(((current->x - NewCell.x)^2) + ((current->y - NewCell.y)^2));
                        NewCell.f = NewCell.g + NewCell.h;
                        NewCell.parent = current;
                        openList.push_back(std::make_shared<Cell>(NewCell));
                        //RCLCPP_INFO(get_logger(),"New created!");
                    }
                    
                }
            }
        
        }
        closedList[current->y*map_.info.width + current->x] = true;
        openList.erase(openList.begin());
    }

    path_.poses.clear();

    if(goalCell)
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        auto cell = goalCell;

        while(cell)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = cell->x*resolution - 7.1;
            pose.pose.position.y = cell->y*resolution - 3.6;
            pose.pose.orientation.w = 1.0; 
            waypoints.push_back(pose);
            cell = cell->parent;
        }
        std::reverse(waypoints.begin(), waypoints.end());
        path_.header.frame_id = "map";
        path_.header.stamp = this->get_clock()->now();
        path_.poses = waypoints;

    }
    else
        RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    
}

void PlanningNode::smoothPath() {
    // add code here
    if (path_.poses.size() < 3) {
        return; // No need to smooth if the path is too short
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    
    int window = 3;
    for (size_t i = 0; i < path_.poses.size(); ++i) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        int count = 0;

        for (int j = -window; j <= window; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < path_.poses.size()) {
                sum_x += path_.poses[idx].pose.position.x;
                sum_y += path_.poses[idx].pose.position.y;
                count++;
            }
        }

        newPath[i].pose.position.x = sum_x / count;
        newPath[i].pose.position.y = sum_y / count;
    }

    path_.poses = newPath;
}

Cell::Cell(int c, int r) 
{
    Cell::x = c;
    Cell::y = r;
}
