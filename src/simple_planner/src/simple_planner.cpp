#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "rp_stuff/grid_map.cpp"
#include <queue>
#include <tf/transform_broadcaster.h>
#include <iostream>

using namespace std;

/*****gloabal variables*****/////
//point in the map
struct point{
  int x;
  int y;
  float cost;
  float heuristic;
  point* parent;
  point(int _x, int _y, double _cost = 0.0, double _heuristic = 0.0, point* _parent = nullptr)//point constructor
        : x(_x), y(_y), cost(_cost), heuristic(_heuristic), parent(_parent) {}

  float cost_Astar() const { return cost + heuristic; } //Note:for A* algorithm f(n)=g(n)+h(n)
};

//initail pose 
geometry_msgs::PoseWithCovarianceStamped start;

//plan publisher
ros::Publisher plan_pub;

// A* path planning algorithm
vector<point*> AStar(const point& start, const point& goal) {
    //TODO
    return vector<point*>();
}

void compute_path(const geometry_msgs::PoseWithCovarianceStamped &start, const geometry_msgs::PoseStamped &goal){
    float start_x=start.pose.pose.position.x;
    float start_y=start.pose.pose.position.y;
    float goal_x=goal.pose.position.x;
    float goal_y=goal.pose.position.y;
    ROS_INFO("Computing path from (%f,%f) to (%f,%f)",start_x,start_y,goal_x,goal_y);
    float resolution=0.01;
    string filename="/home/angelo/catkin_ws/src/simple_planner/src/maze.jpg";

    //initialize grid_map
    GridMap grid_map(0, 0, 0.01);
    grid_map.loadFromImage(filename.c_str(), resolution);

    // Run A* algorithm
    point start_cell(start_x, start_y);
    point goal_cell(goal_x, goal_y);
    std::vector<point*> path = AStar(start_cell, goal_cell);

    // Create a nav_msgs::Path message
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map"; // Set the frame ID of the path

    // Add poses to the path message
    for (const auto& point : path) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = point->x;
      pose.pose.position.y = point->y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = ros::Time::now(); // Timestamp for each point in the path
      pose.header.frame_id = "map"; // Set the frame ID of the pose
      path_msg.poses.push_back(pose);
    }

    //publishing path to rviz
    ROS_INFO("Path Computed,publishing path!");
    plan_pub.publish(path_msg);
}

//callback functions
//initial pose callback
void handle_initial(const geometry_msgs::PoseWithCovarianceStamped &pose){
  start=pose;
  ROS_INFO("Got initial pose!");
}

//goal pose callback
void handle_goal(const geometry_msgs::PoseStamped &goal){
  ROS_INFO("Got goal pose!");
  ROS_INFO("Computing path with A* search algorithm");
  compute_path(start,goal);
}

int main(int argc, char **argv) {
  //initialize ros
  ros::init(argc, argv, "simple_planner");

  //defince necessary ros nodes
  ros::NodeHandle init_node;
  ros::NodeHandle goal_node;
  ros::NodeHandle plan_node;

  //define required subscribers and publisher
  ros::Subscriber initial_sub = init_node.subscribe("initialpose", 10, handle_initial);
  ros::Subscriber goal_sub = goal_node.subscribe("move_base_simple/goal", 10, handle_goal);
  plan_pub = plan_node.advertise<nav_msgs::Path>("nav_msgs/Path", 1);

  //start ros
  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}