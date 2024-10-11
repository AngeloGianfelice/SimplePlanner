#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include <queue>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/package.h>

using namespace std;
using namespace cv;

/*****gloabal variables*****/
int dis_fun = 0; // default = 0 = euclidean distance
string map_filename = "maze.jpg";
float resolution = 0.05;
float origin_x= -5.0;
float origin_y= -5.0;
int my_threshold = 127;
int rows,cols;
ros::Publisher plan_pub;

//point in the map
struct point {
    int x,y;
    float c_cost,h_cost;
    point* parent;

    point(int x_, int y_, float c_cost_, float h_cost_, point* parent_ = nullptr){ //point constructor
        x=x_;
        y=y_;
        c_cost=c_cost_;
        h_cost=h_cost_; //Note:for A* algorithm f(n)=h(n)+g(n)
        parent=parent_;
    }

    float f_cost() const { return c_cost + h_cost; }

    bool operator<(const point& other) const {
        return this->f_cost() > other.f_cost();
    }
};

//path grid coordinates (initialized to -1)
struct path{

    int start_x;
    int start_y;
    int goal_x;
    int goal_y;

} my_path = {-1,-1,-1,-1};

/*****HELPER FUNCTIONS*****/

//Heuristic funtion
double distance(int x1, int y1, int x2, int y2) {

    // Euclidean distance
    if(dis_fun == 0) {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    }else{
        //Manhattan distance
        return abs(x1 - x2) + abs(y1 - y2);
    }
}

// Convert grid to world coordinates
void gridToWorld(int grid_x, int grid_y, float& world_x, float& world_y) {

    world_x = origin_x + (grid_x * resolution);
    world_y = origin_y + (grid_y * resolution);
}

// Convert world to grid coordinates
void worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) {

    grid_x = (world_x - origin_x) / resolution;
    grid_y = (world_y - origin_y) / resolution;
}

//Check if point is traversable
bool isInside(int x, int y){

    if(x >= 0 && y >= 0 && x < rows && y < cols){
        return true;
    }
    else return false; 
}

// A* path planning algorithm
void AStar(path my_path, vector<vector<int>> grid){

    vector<vector<bool>> visited(rows, vector<bool>(cols, false)); //visited list

    priority_queue<point> open_list;

    // Start point
    point* start = new point(my_path.start_x, my_path.start_y, 0.0, distance(my_path.start_x, my_path.start_y, my_path.goal_x, my_path.goal_y));
    open_list.push(*start);

    // Directions for 4-connected grid
    vector<pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

    point* goal_point = nullptr;
 
    while (!open_list.empty()) {

        point current = open_list.top();
        open_list.pop();

        // Check if goal is reached
        if (current.x == my_path.goal_x && current.y == my_path.goal_y) {

            //path reconstruction
            goal_point = new point(current);
            break;
        }

        //skip point if already visited
        if (visited[current.x][current.y]) continue;

        // Explore neighbors
        for (const auto& direction : directions) {

            int new_x = current.x + direction.first;
            int new_y = current.y + direction.second;

            // Check if point is traversable and not already visited
            if (isInside(new_x,new_y) && grid[new_x][new_y] == 0 && !visited[new_x][new_y]) {

                float new_c_cost = current.c_cost + 1.0;
                point* new_point = new point(new_x,new_y,new_c_cost,distance(new_x,new_y,my_path.goal_x,my_path.goal_y), new point(current));
                open_list.push(*new_point);

            }

        }

        //marked current point as visited
        visited[current.x][current.y] = true;

    }

    if (goal_point) {

            // Reconstruct the path
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "map";

            point* curr = goal_point;
            while (curr) {
                geometry_msgs::PoseStamped pose;
                float world_x,world_y;
                gridToWorld(curr->x,curr->y,world_x,world_y);
                pose.pose.position.x = world_x;
                pose.pose.position.y = world_y;
                path_msg.poses.push_back(pose);
                curr = curr->parent;
            }
            plan_pub.publish(path_msg);
            ROS_INFO("Path Published!");
    } else {
        ROS_WARN("No valid path found!");
    }

}

vector<vector<int>> getOccupancyGrid(){

    string filename = ros::package::getPath("simple_planner") + "/src/maps/" + map_filename;
    //getting input image (a.k.a. the map)
    Mat image = imread(filename.c_str(),IMREAD_GRAYSCALE);
    if(image.empty()){

        ROS_ERROR("Failed to load image!!");
        return vector<vector<int>>();

    }
    
    //Costructing occupancy grid from input image
    Mat binary,rotated;
    threshold(image,binary,my_threshold,255,THRESH_BINARY);
    rotate(binary,rotated,ROTATE_90_CLOCKWISE);
    vector<vector<int>> occupancy_grid = vector<vector<int>>(rotated.rows, vector<int>(rotated.cols,0));

    for(int i=0;i<rotated.rows;i++){

        for(int j=0;j<rotated.cols;j++){

            occupancy_grid[i][j] = (rotated.at<uchar>(i,j) == 255) ? 0 : 1; // check if point is obstacle

        }

    }

    return occupancy_grid;
}

void compute_path(){

    ROS_INFO("Computing path from Grid: (%d,%d) to (%d,%d)",my_path.start_x,my_path.start_y,my_path.goal_x,my_path.goal_y);

    //Constructing Occupancy Grid form input image map
    vector<vector<int>> grid = getOccupancyGrid();
    
    rows = grid.size();
    cols = (rows > 0) ? grid[0].size() : 0;

    //starting search algorithm
    AStar(my_path,grid);

}


//callback functions
//initial pose callback
void handle_initial(const geometry_msgs::PoseWithCovarianceStamped &start){

    //getting world coordinates from start pose
    float start_world_x=start.pose.pose.position.x;
    float start_world_y=start.pose.pose.position.y;

    //Converting world coordinates into grid coordinates
    int start_x,start_y;
    worldToGrid(start_world_x,start_world_y,start_x,start_y);

    //storing start coordinates into path struct
    my_path.start_x = start_x;
    my_path.start_y = start_y;

    ROS_INFO("Got initial pose!");

}

//goal pose callback
void handle_goal(const geometry_msgs::PoseStamped &goal){

    //getting world coordinates from goal pose
    float goal_world_x=goal.pose.position.x;
    float goal_world_y=goal.pose.position.y;

    //Converting world coordinates into grid coordinates
    int goal_x,goal_y;
    worldToGrid(goal_world_x,goal_world_y,goal_x,goal_y);
    
    //storing start coordinates into path struct
    my_path.goal_x = goal_x;
    my_path.goal_y = goal_y;
    
    ROS_INFO("Got goal pose!");

    //we are ready to plan our path
    compute_path();

}

int main(int argc, char **argv) {

    // Initialize the ROS 
    ros::init(argc, argv, "simple_planner");

    
    //input parameters handling
    string distance;
    string filename;

    if(ros::param::get("/distance",distance)){
        ROS_INFO("distance function: %s",distance.c_str());
        if(distance == "manhattan"){
            dis_fun = 1;
        }else if(distance == "euclidean"){
            dis_fun = 0;
        }else{
            ROS_WARN("wrong distance function, using default(euclidean)");
        }
    }else{
        ROS_WARN("missing distance function, using default(euclidean)");
    }

    if(ros::param::get("/filename",filename)){
        ROS_INFO("filename: %s",filename.c_str());
        map_filename = filename;
    }else{
        ROS_WARN("map filename not provided, using default map");
    }

    if(ros::param::get("/threshold",my_threshold)){
        ROS_INFO("threshold: %d",my_threshold);
        if(my_threshold < 0 || my_threshold > 256){
            ROS_WARN("given threshold is not in the range (0,256), using default(127)");
        }
    }else{
        ROS_WARN("threshold not valid, using default(127)");
    }

    //defince necessary ros nodeHandles
    ros::NodeHandle init_node;
    ros::NodeHandle goal_node;
    ros::NodeHandle plan_node;

    //define required subscribers and publisher
    ros::Subscriber initial_sub = init_node.subscribe("initialpose", 10, handle_initial);
    ros::Subscriber goal_sub = goal_node.subscribe("move_base_simple/goal", 10, handle_goal);
    plan_pub = plan_node.advertise<nav_msgs::Path>("nav_msgs/Path", 1);

    //ROS loop
    ros::Rate loop_rate(1);
    while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}