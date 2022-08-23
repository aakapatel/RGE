#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>
#include <ros/ros.h>
#include <list>
#include <iostream>
#include <chrono>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <tf/tf.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <acl_exploration/ServerConfig.h>
#include "exploration/Frontiers.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace std::chrono;
using namespace std;
using namespace std::this_thread;

// Structs


struct Frontier_main {
  std::list<ufo::math::Vector3> list;
  ufo::math::Vector3 position;
  double heading;
};

struct Graph {
  std::pair<ufo::math::Vector3, ufo::math::Vector3> pair;
  double pair_wt;
};

// Definition of Structure "node" and an array to store inputs
struct node {
        int x; // x-cordinate of dot
        int y; // y-cordinate of dot
        int z; // y-cordinate of dot
}; 

// Definition of Structure "edge"
// struct edge {
//          ufo::math::Vector3 t1; // index of dot in dots[] for an end.
//          ufo::math::Vector3 t2; // index of dot in dots[] for another end.
//          double weight; // weight (geometric distance between two ends) 
// };


// Variables

float SCALER_AABB;
float SCALER_X = SCALER_AABB;
float SCALER_Y = SCALER_AABB;
float SCALER_Z = SCALER_AABB;
float position_x = 0;
float position_y = 0;
float position_z = 0;
float velocity_x = 0;
float velocity_y = 0;
float velocity_z = 0;
bool map_received = false;
bool position_received;
float SENSOR_RANGE;
float SENSOR_MIN;
float SENSOR_VERTICAL;
float SENSOR_HORIZONTAL;
float SCALER_INFORMATION_GAIN;
float SCALER_DISTANCE;
float SCALER_ACTUATION;
float NEXT_POINT_DISTANCE;
float NEXT_PATH_DISTANCE;
double RADIOUS;
double RESOLUTION;
double roll = 0;
double pitch = 0;
double yaw = 0;
double md;
double head_ref;
bool firstTime = true;
bool start_flag = true;
bool move_flag = false;
// double heading = 0;

bool frontiers_received;
Frontier_main all;
Frontier_main local;
Frontier_main global;
Frontier_main current;
Frontier_main old_poses;
Graph edge;
ufo::math::Vector3 selectedfrontier;
ufo::math::Vector3 selectedgfrontier;
ufo::math::Vector3 nearest_to_global;
geometry_msgs::PoseStamped best_frontier;
nav_msgs::Odometry m_currentPose;
nav_msgs::Odometry m_prevPose;
// edge lines[10000];


ufo::map::OccupancyMapColor myMap(RESOLUTION);

// Lists
// std::list<ufo::math::Vector3> extracted;
std::list<ufo::math::Vector3> ext;
std::list<ufo::math::Vector3> accessible;
std::list<ufo::math::Vector3> positionslist;

// Functions


// Collision checks

bool isInCollision(ufo::map::OccupancyMapColor const& map, 
                   ufo::geometry::BoundingVar const& bounding_volume, 
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = 3)
{
  // Iterate through all leaf nodes that intersects the bounding volume
  for (auto it = map.beginLeaves(bounding_volume, occupied_space, 
                                 free_space, unknown_space, false, min_depth), 
        it_end = map.endLeaves(); it != it_end; ++it) {
    // Is in collision since a leaf node intersects the bounding volume.
    return true;
  }
  // No leaf node intersects the bounding volume.
  return false;
}

double calculateDistance(ufo::math::Vector3 point1, ufo::math::Vector3 point2) {

  double dist = abs(sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) 
                                                          + pow(point1.y() - point2.y(), 2)));
  
  return dist;

}



std::list<ufo::math::Vector3> extractUnknowns(ufo::map::OccupancyMapColor const& map, 
                  std::list<ufo::math::Vector3> flist, ufo::math::Vector3 position,
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = 0)
{
  std::list<ufo::math::Vector3> extracted;


    for (ufo::math::Vector3 goal : flist) {


      // The direction from position to goal
      ufo::math::Vector3 direction = goal - position;
      // The center between position and goal
      ufo::math::Vector3 center = position + (direction / 2.0);
      // The distance between position and goal
      double distance = direction.norm();
      // Normalize direction
      direction /= distance;

      // Calculate yaw, pitch and roll for oriented bounding box
      double yaw = -atan2(direction[1], direction[0]);
      double pitch = -asin(direction[2]);
      double roll = 0;  // TODO: Fix
      
      // Create an oriented bounding box between position and goal, with a size radius.
      ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.25, 0.25),
              ufo::math::Quaternion(roll, pitch, yaw));

      // Check if the oriented bounding box collides with either occupied or unknown space,
      // at depth 3 (16 cm)
      if (!isInCollision(map, obb, true, false, false, 0)) {

        extracted.push_back(goal);

      }      
      

    }

  return extracted;

}

double calculateAngle(ufo::math::Vector3 fpoint, ufo::math::Vector3 position,
                                                double heading) {

  double head = atan2((fpoint.y() - position.y()), (fpoint.x() - position.x())) * (180/3.14159);
  double angle = head - heading;

  if (head > -180 && head < -90) {
          if (heading > -180 && heading < -90) {                
              angle = head - heading;
              if (angle > 180) {
                  angle = angle - 360;
              }
          } 
          else {
              angle = angle + 360;
              if (angle > 180) {
                  angle = angle - 360;
              }                            
          }
  } else if (head > 0) {
      if (heading > -180 && heading < -90) {
          angle = angle - 360;
          if (angle > 180) {
              angle = angle - 360;
          }
      } else {
          angle = angle;
          if (angle > 180) {
              angle = angle - 360;
          }
      }
  }

  return angle;

}

bool isLocal(ufo::math::Vector3 front, ufo::math::Vector3 position,
                                                              double heading) {

  // for (ufo::math::Vector3 front : allfrontiers) {

  double angle = calculateAngle(front, position, heading);
  double distance = calculateDistance(front, position);

    if (((angle < 40) && (angle > -40)) && distance < 10) {

      return true;

    } else {

      return false;
    }

  // }

}

ufo::math::Vector3 selectLocalFrontier(std::list<ufo::math::Vector3> accessible, 
                                                ufo::math::Vector3 position, double heading) {

  double cost;
  double mycost = 100000000;
  ufo::math::Vector3 select;
  for (ufo::math::Vector3 localfrontier : accessible) {

    double distance = calculateDistance(localfrontier, position);
    double angle = calculateAngle(localfrontier, position, heading);

    // if (((angle < 10) && (angle > -10)) && distance < 10) {

      cost = angle;
      if ( (localfrontier.z() > 3) || (localfrontier.z() < 0.8)) {
          cost = cost + 10000;
      } else {
          cost = cost;
      }

      if (cost < 0) {
          cost = -cost;          
          if (cost < mycost) {             
              mycost = cost;
              select = localfrontier;
          }
      }
      if (cost > 0) {          
          if (cost < mycost) {              
              mycost = cost;
              select = localfrontier;
          }
      }
    // }
  }
  return select;
}

ufo::math::Vector3 selectGlobalFrontier(std::list<ufo::math::Vector3> global, 
                                                ufo::math::Vector3 position, double heading) {

  double score;
  double myScore = 100000000;
  ufo::math::Vector3 selectg;
  for (ufo::math::Vector3 globalfrontier : global) {

    double distance = calculateDistance(globalfrontier, position);

      score = distance;
      if ( (globalfrontier.z() > 3) || (globalfrontier.z() < 0.8)) {
          score = score + 1000;
      } else {
          score = score;
      }

      if (score < myScore) {              
          myScore = score;
          selectg = globalfrontier;
      }
  }
  return selectg;
}

bool next_pub(geometry_msgs::PoseStamped pose, ufo::math::Vector3 position) {

  ufo::math::Vector3 pos;
  pos.x() = pose.pose.position.x;
  pos.y() = pose.pose.position.y;
  pos.z() = pose.pose.position.z;

  double pub_dis = calculateDistance(pos, position);
  if (pub_dis < 2) {
    return true;
  } else {
    return false;
  }

}

ufo::math::Vector3 visitedToGlobal(std::list<ufo::math::Vector3> visited, ufo::math::Vector3 global_fronier,
                                          ufo::map::OccupancyMapColor const& map) {
  double num;
  double mynum = 10000000;
  ufo::math::Vector3 cp;
  std::list<ufo::math::Vector3>::iterator it;
  for (it = visited.begin(); it != visited.end(); it++) {


    // The direction from position to goal
    ufo::math::Vector3 direction = *it - global_fronier;
    // The center between position and goal
    ufo::math::Vector3 center = global_fronier + (direction / 2.0);
    // The distance between position and goal
    double distance = direction.norm();
    // Normalize direction
    direction /= distance;

    // Calculate yaw, pitch and roll for oriented bounding box
    double yaw = -atan2(direction[1], direction[0]);
    double pitch = -asin(direction[2]);
    double roll = 0;  // TODO: Fix
    
    // Create an oriented bounding box between position and goal, with a size radius.
    ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.25, 0.25),
            ufo::math::Quaternion(roll, pitch, yaw));


    double m = calculateDistance(*it, global_fronier);
    num = m;
    if (num < mynum && !isInCollision(map, obb, true, false, false, 0)) {
      mynum = num;
      cp = *it;
    }
  }
  return cp;

}

// Function to create edges of complete graph from an array of nodes.
//   Argument: number of nodes stored in the array dots.
//   ReturnValue: number of edges created.
//   Assumption: the array lines is large enough to store all edges of complete graph
// std::vector<std::pair<std::pair<ufo::math::Vector3, ufo::math::Vector3>, double>>
std::vector<std::pair<std::pair<ufo::math::Vector3, ufo::math::Vector3>, double>> createCompleteGraph(
                          std::list<ufo::math::Vector3> oldlist, ufo::map::OccupancyMapColor const& map){
  int k;
  std::vector<std::pair<std::pair<ufo::math::Vector3, ufo::math::Vector3>, double>> myvec;
  k=0; // k is index of array lines

  std::list<ufo::math::Vector3>::iterator it;
  
  for (it = oldlist.begin(); it != std::prev(oldlist.end(), 1); it++) {
    std::list<ufo::math::Vector3>::iterator it2;
    for (it2 = std::next(it, 1); it2 != oldlist.end(); it2++) {
      

      ufo::math::Vector3 direction = *it - *it2;
      // The center between position and goal
      ufo::math::Vector3 center = *it2 + (direction / 2.0);
      // The distance between position and goal
      double distance = direction.norm();
      // Normalize direction
      direction /= distance;

      // Calculate yaw, pitch and roll for oriented bounding box
      double yaw = -atan2(direction[1], direction[0]);
      double pitch = -asin(direction[2]);
      double roll = 0;  // TODO: Fix
      
      // Create an oriented bounding box between position and goal, with a size radius.
      ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.5, 0.5),
              ufo::math::Quaternion(roll, pitch, yaw));

      if (!isInCollision(map, obb, true, false, false, 0)) {
      
        myvec.insert(myvec.end(), std::make_pair(std::make_pair(*it, *it2), calculateDistance(*it, *it2)));

      }
    }
  }
return myvec;
}





// End of functions
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Callback functions

void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const& msg)
{
  if(ufomap_msgs::msgToUfo(msg->map, myMap)) {
    map_received = true;
  } else {
    ROS_INFO("Map conversion failed");
  }
}

void config_callback(acl_exploration::ServerConfig &config, uint32_t level)
{
	// Read parameters
	RADIOUS = config.Radious;
	SCALER_AABB = config.Bounding_box_radious;
	SCALER_DISTANCE = config.Scaler_distance_cost;
	SCALER_INFORMATION_GAIN = config.Scaler_information_gain;
	SCALER_ACTUATION = config.Scaler_actuation_cost;
	SENSOR_RANGE = config.Sensor_range;
	SENSOR_MIN = config.Sensor_min;
	SENSOR_HORIZONTAL = config.Sensor_angle_horizontal;
	SENSOR_VERTICAL = config.Sensor_angle_vertical;
	RESOLUTION = config.Resolution;
	
	
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  position_received = true;
  ufo::math::Vector3 oldPoint;
  ufo::math::Vector3 newPoint;
  
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  position_z = msg->pose.pose.position.z;
  velocity_x = msg->twist.twist.linear.x;
  velocity_y = msg->twist.twist.linear.y;
  velocity_z = msg->twist.twist.linear.z;

  if (start_flag) {
    m_prevPose = *msg;
    start_flag = false;
  } else {

    m_currentPose = *msg;
    newPoint = ufo::math::Vector3(msg->pose.pose.position.x, 
                                      msg->pose.pose.position.y, 
                                            msg->pose.pose.position.z);
    oldPoint = ufo::math::Vector3(m_prevPose.pose.pose.position.x, 
                                      m_prevPose.pose.pose.position.y, 
                                            m_prevPose.pose.pose.position.z);

    
    if (calculateDistance(newPoint, oldPoint) > 2) {
      m_prevPose = *msg;
      positionslist.push_back(oldPoint);
      
    }
    if (calculateDistance(newPoint, oldPoint) > 1.8) {
      move_flag = true;
      
      
    } else {
      move_flag = false;
    }

    old_poses.list = positionslist;

  }

  current.position = ufo::math::Vector3(position_x, position_y, position_z);
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  current.heading = yaw * (180/3.14159);
  
  
}

void frontiersCallback(const exploration::Frontiers::ConstPtr& msg) {

  frontiers_received = true;
  ufo::math::Vector3 frontier_point;
  
  std::list<ufo::math::Vector3> mylist;
  std::list<ufo::math::Vector3> mylist1;
  std::list<ufo::math::Vector3> mylist2;
  for (geometry_msgs::Point frontier : msg->points) {

    frontier_point = ufo::math::Vector3(frontier.x, frontier.y, frontier.z);
    mylist.push_back(frontier_point);
    all.list = mylist;
    // std::cout << "the frontiers list size is: " << frontiers.list.size() << std::endl;
    if (isLocal(frontier_point, current.position, current.heading)) {
      mylist1.push_back(frontier_point);
      local.list = mylist1;
    } else {
      mylist2.push_back(frontier_point);
      global.list = mylist2;
    }
  }

}

// Publish all visualization messages here.

void visualize(ros::Publisher* position_pub, ros::Publisher* points_pub, ros::Publisher* map_pub, 
                                                      ros::Publisher* hits_pub, ros::Publisher* local_nodes_pub,
                                                              ros::Publisher global_connection_visualization_pub,
                                                                    ros::Publisher global_connection_line_pub){
  visualization_msgs::Marker POSITION_point, HITS_points, local_frontier_nodes, global_connection_points, global_connection_line_list;

  if(old_poses.list.size() >= 0){
    POSITION_point.header.frame_id = "world";
    POSITION_point.ns = "points";
    POSITION_point.action = visualization_msgs::Marker::ADD;
    POSITION_point.pose.orientation.w = 1.0;
    POSITION_point.id = 0;
    POSITION_point.type = visualization_msgs::Marker::POINTS;
    POSITION_point.scale.x = 0.25;
    POSITION_point.scale.y = 0.25;
    // POSITION_point.scale.z = 0.25;
    POSITION_point.color.r = 1.0f;
    POSITION_point.color.g = 1.0f;
    POSITION_point.color.b = 1.0f;
    POSITION_point.color.a = 1.0;
    std::list<ufo::math::Vector3>::iterator it_comeon_visualizer1;	
    for(it_comeon_visualizer1 = old_poses.list.begin(); it_comeon_visualizer1 != old_poses.list.end(); it_comeon_visualizer1++){
      geometry_msgs::Point p;
      p.x = it_comeon_visualizer1->x();
      p.y = it_comeon_visualizer1->y();
      p.z = it_comeon_visualizer1->z();
      POSITION_point.points.push_back(p);
    }
    position_pub->publish(POSITION_point);
    
  }


  if(ext.size() != 0){
    HITS_points.header.frame_id = "world";
    HITS_points.ns = "points";
    HITS_points.action = visualization_msgs::Marker::ADD;
    HITS_points.pose.orientation.w = 1.0;
    HITS_points.id = 0;
    HITS_points.type = visualization_msgs::Marker::POINTS;
    HITS_points.scale.x = 0.2;
    HITS_points.scale.y = 0.2;
    HITS_points.color.r = 1.0f;
    HITS_points.color.a = 1.0;
    std::list<ufo::math::Vector3>::iterator it_comeon_visualizer2;	
    for(it_comeon_visualizer2 = ext.begin(); it_comeon_visualizer2 != ext.end(); it_comeon_visualizer2++){
      geometry_msgs::Point p;
      p.x = it_comeon_visualizer2->x();
      p.y = it_comeon_visualizer2->y();
      p.z = it_comeon_visualizer2->z();
      HITS_points.points.push_back(p);
    }
    hits_pub->publish(HITS_points);
  }

  if(local.list.size() != 0){
    local_frontier_nodes.header.frame_id = "world";
    local_frontier_nodes.ns = "points";
    local_frontier_nodes.action = visualization_msgs::Marker::ADD;
    local_frontier_nodes.pose.orientation.w = 1.0;
    local_frontier_nodes.id = 0;
    local_frontier_nodes.type = visualization_msgs::Marker::POINTS;
    local_frontier_nodes.scale.x = 0.4;
    local_frontier_nodes.scale.y = 0.4;
    local_frontier_nodes.color.g = 0.7f;
    local_frontier_nodes.color.a = 0.6;
    std::list<ufo::math::Vector3>::iterator it_comeon_visualizer3;	
    for(it_comeon_visualizer3 = local.list.begin(); it_comeon_visualizer3 != local.list.end(); it_comeon_visualizer3++){
      geometry_msgs::Point p;
      p.x = it_comeon_visualizer3->x();
      p.y = it_comeon_visualizer3->y();
      p.z = it_comeon_visualizer3->z();
      local_frontier_nodes.points.push_back(p);
    }
    local_nodes_pub->publish(local_frontier_nodes);
  }


  if(old_poses.list.size() >= 0){
    global_connection_points.header.frame_id = global_connection_line_list.header.frame_id = "world";
    global_connection_points.ns = "points";
    global_connection_points.action = visualization_msgs::Marker::ADD;
    global_connection_points.pose.orientation.w = 1.0;
    global_connection_points.id = 0;
    global_connection_line_list.id = 1;
    global_connection_points.type = visualization_msgs::Marker::POINTS;
    global_connection_line_list.type = visualization_msgs::Marker::LINE_LIST;
    global_connection_points.scale.x = 0.2;
    global_connection_points.scale.y = 0.2;
    global_connection_line_list.scale.x = 0.05;
    global_connection_points.color.b = 1.0f;
    global_connection_points.color.a = 1.0;
    global_connection_line_list.color.r = 1.0;
    global_connection_line_list.color.g = 0.75;
    global_connection_line_list.color.b = 0.80;
    global_connection_line_list.color.a = 1.0;
    // selectedgfrontier = selectGlobalFrontier(global.list, current.position, current.heading);
    // nearest_to_global = visitedToGlobal(old_poses.list, selectedgfrontier, myMap);
    std::vector<std::pair<std::pair<ufo::math::Vector3, ufo::math::Vector3>, double>> mp = createCompleteGraph(old_poses.list, myMap);
    geometry_msgs::Point p1;
      p1.x = nearest_to_global.x();
      p1.y = nearest_to_global.y();
      p1.z = nearest_to_global.z();
      global_connection_points.points.push_back(p1);
      
        for (auto i = mp.begin(); i != mp.end(); i++) {

        
          // global_connection_line_list.points.push_back(p2);
          geometry_msgs::Point p2;
          p2.x = ((*i).first).first.x();
          p2.y = ((*i).first).first.y();
          p2.z = ((*i).first).first.z();
          global_connection_line_list.points.push_back(p2);
          geometry_msgs::Point p3;
          p3.x = ((*i).first).second.x();
          p3.y = ((*i).first).second.y();
          p3.z = ((*i).first).second.z();
          global_connection_line_list.points.push_back(p3);
        }
        
      
    
    global_connection_visualization_pub.publish(global_connection_points);
    global_connection_line_pub.publish(global_connection_line_list);
  }



}


// End of callback functions
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Main


int main(int argc, char *argv[])
{ 
  // Subscribers and publishers
  ros::init(argc, argv, "ACL_EXPLORATION");
  ros::NodeHandle nh;
  ros::Publisher points_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes", 1);
  ros::Publisher position_pub = nh.advertise<visualization_msgs::Marker>("POSITION", 1);
  ros::Publisher best_frontier_pub = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird/reference", 1);
  ros::Publisher global_connection_visualization_pub = nh.advertise<visualization_msgs::Marker>("/points_global_connection", 1);
  ros::Publisher global_connection_line_pub = nh.advertise<visualization_msgs::Marker>("/line_global_connection", 1);
  ros::Publisher hits_pub = nh.advertise<visualization_msgs::Marker>("Traversable_frontiers", 1);
  ros::Publisher local_nodes_pub = nh.advertise<visualization_msgs::Marker>("local_frontier_nodes_vis", 1);
  ros::Publisher map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("acl_full_map", 11);
  ros::Subscriber map_sub = nh.subscribe("ufomap_mapping_server_node/map", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("/hummingbird/ground_truth/odometry", 1, odomCallback);
  ros::Subscriber front_sub = nh.subscribe("/frontiers_nodes", 1, frontiersCallback);
  // ros::Subscriber sub = nh.subscribe("/odometry/imu", 1, odomCallback);

  ros::Rate rate(10);
  
  // Dynamic reconfigure
  dynamic_reconfigure::Server<acl_exploration::ServerConfig> server;
  dynamic_reconfigure::Server<acl_exploration::ServerConfig>::CallbackType f;
  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);
  

  
  // Main


  while(ros::ok()){
    high_resolution_clock::time_point start_total = high_resolution_clock::now();


    /*
      main computations to be executed here

    */


    ext = extractUnknowns(myMap, local.list, current.position, true, true, true, 0);
    cout << "Accessible frontiers array size is " << ext.size() << endl;
    // cout << "Local frontiers array size is " << local.list.size() << endl;
    // cout << "\nGlobal frontiers array size is " << global.list.size() << endl;
    
    // cout << "poselist size is " << positionslist.size() << endl;
    // cout << "old poses array size is " << old_poses.list.size() << endl;

    // if (old_poses.list.size() >= 2) {
    //   std::vector<std::pair<std::pair<ufo::math::Vector3, ufo::math::Vector3>, double>> mp = createCompleteGraph(old_poses.list);
    //   std::cout << "number of possible edges are: " << ((mp[0]).first).first.x() << std::endl;
    // }


    // check the number of edges here 
    if (firstTime) {
      firstTime = false;
      selectedfrontier = selectLocalFrontier(ext, current.position, current.heading);
      head_ref = atan2((selectedfrontier.y() - current.position.y()), (selectedfrontier.x() - current.position.x()));
      best_frontier.pose.position.x = selectedfrontier.x();
      best_frontier.pose.position.y = selectedfrontier.y();
      best_frontier.pose.position.z = selectedfrontier.z();
      best_frontier.pose.orientation.x = 0;
      best_frontier.pose.orientation.y = 0;
      best_frontier.pose.orientation.z = head_ref;
      best_frontier.pose.orientation.w = 0;
      best_frontier_pub.publish(best_frontier);
      ROS_INFO("Local Frontier Published");
    }

    if (next_pub(best_frontier, current.position)) {

      if (ext.size() > 2) {
      
          selectedfrontier = selectLocalFrontier(ext, current.position, current.heading);
          cout << "local frontier X: " << selectedfrontier.x() << endl;
          cout << "local frontier Y: " << selectedfrontier.y() << endl;
          cout << "local frontier Z: " << selectedfrontier.z() << endl;
          head_ref = atan2((selectedfrontier.y() - current.position.y()), (selectedfrontier.x() - current.position.x()));
          best_frontier.pose.position.x = selectedfrontier.x();
          best_frontier.pose.position.y = selectedfrontier.y();
          best_frontier.pose.position.z = selectedfrontier.z();
          best_frontier.pose.orientation.x = 0;
          best_frontier.pose.orientation.y = 0;
          best_frontier.pose.orientation.z = head_ref;
          best_frontier.pose.orientation.w = 0;
          best_frontier_pub.publish(best_frontier);
          ROS_INFO("Local Frontier Published");
      } else {
          selectedgfrontier = selectGlobalFrontier(global.list, current.position, current.heading);
          nearest_to_global = visitedToGlobal(old_poses.list, selectedgfrontier, myMap);
          cout << "global frontier X: " << selectedgfrontier.x() << endl;
          cout << "global frontier Y: " << selectedgfrontier.y() << endl;
          cout << "global frontier Z: " << selectedgfrontier.z() << endl;
          head_ref = atan2((selectedgfrontier.y() - current.position.y()), (selectedgfrontier.x() - current.position.x()));
          best_frontier.pose.position.x = selectedgfrontier.x();
          best_frontier.pose.position.y = selectedgfrontier.y();
          best_frontier.pose.position.z = selectedgfrontier.z();
          best_frontier.pose.orientation.x = 0;
          best_frontier.pose.orientation.y = 0;
          best_frontier.pose.orientation.z = head_ref;
          best_frontier.pose.orientation.w = 0;
          best_frontier_pub.publish(best_frontier);
          ROS_INFO("Global Frontier Published");
      }

    }
    
    // cout << "Selected local frontier X: " << selectedlocalfrontier.x() << endl;
    // cout << "Selected local frontier Y: " << selectedlocalfrontier.y() << endl;
    // cout << "Selected local frontier Z: " << selectedlocalfrontier.z() << endl;
    visualize(&points_pub, &map_pub, &position_pub, &hits_pub, &local_nodes_pub, global_connection_visualization_pub,
                                                                    global_connection_line_pub);
    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    cout << "\nExecution time: " << duration_total.count() << " ms " << endl;
    
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// TODO LIST

/*

1) Fix frontiers detection -- done
2) Compute information gain -- in process
3) Fix heading deviation based search 
4) Active loop closing 

*/
