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
double head_ref;
bool firstTime = true;
// double heading = 0;

bool frontiers_received;
Frontier_main all;
Frontier_main local;
Frontier_main global;
Frontier_main current;
ufo::math::Vector3 selectedfrontier;
ufo::math::Vector3 selectedgfrontier;
geometry_msgs::PoseStamped best_frontier;

ufo::map::OccupancyMapColor myMap(RESOLUTION);

// Lists
// std::list<ufo::math::Vector3> extracted;
std::list<ufo::math::Vector3> ext;
std::list<ufo::math::Vector3> accessible;

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

    if (((angle < 40) && (angle > -40)) && distance < 10) {

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
    }
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




// Publish all visualization messages here.

void visualize(ros::Publisher* position_pub, ros::Publisher* points_pub, ros::Publisher* map_pub, 
                                                      ros::Publisher* hits_pub, ros::Publisher* local_nodes_pub){
  visualization_msgs::Marker POSITION_point, HITS_points, local_frontier_nodes;

  if(position_received){
    POSITION_point.header.frame_id = "world";
    POSITION_point.ns = "points";
    POSITION_point.action = visualization_msgs::Marker::ADD;
    POSITION_point.pose.orientation.w = 1.0;
    POSITION_point.id = 0;
    POSITION_point.type = visualization_msgs::Marker::POINTS;
    POSITION_point.scale.x = 0.2;
    POSITION_point.scale.y = 0.2;
    POSITION_point.color.g = 1.0f;
    POSITION_point.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = position_x;
    p.y = position_y;
    p.z = position_z;
    POSITION_point.points.push_back(p);
    position_pub->publish(POSITION_point);
  }


  if(ext.size() != 0){
    // extracted.clear();
    // goalNode->addHits(&hits);
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
    std::list<ufo::math::Vector3>::iterator it_comeon_visualizer4;	
    for(it_comeon_visualizer4 = ext.begin(); it_comeon_visualizer4 != ext.end(); it_comeon_visualizer4++){
      geometry_msgs::Point p;
      p.x = it_comeon_visualizer4->x();
      p.y = it_comeon_visualizer4->y();
      p.z = it_comeon_visualizer4->z();
      HITS_points.points.push_back(p);
    }
    hits_pub->publish(HITS_points);
  }

  if(local.list.size() != 0){
    // extracted.clear();
    // goalNode->addHits(&hits);
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
    std::list<ufo::math::Vector3>::iterator it_comeon_visualizer4;	
    for(it_comeon_visualizer4 = local.list.begin(); it_comeon_visualizer4 != local.list.end(); it_comeon_visualizer4++){
      geometry_msgs::Point p;
      p.x = it_comeon_visualizer4->x();
      p.y = it_comeon_visualizer4->y();
      p.z = it_comeon_visualizer4->z();
      local_frontier_nodes.points.push_back(p);
    }
    local_nodes_pub->publish(local_frontier_nodes);
  }
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
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  position_z = msg->pose.pose.position.z;
  velocity_x = msg->twist.twist.linear.x;
  velocity_y = msg->twist.twist.linear.y;
  velocity_z = msg->twist.twist.linear.z;
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


// End of callback functions
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Main


int main(int argc, char *argv[])
{ 
  // Subscribers and publishers
  ros::init(argc, argv, "ACL_EXPLORATION");
  ros::NodeHandle nh;
  ros::Publisher points_pub = nh.advertise<visualization_msgs::Marker>("NODES", 1);
  ros::Publisher position_pub = nh.advertise<visualization_msgs::Marker>("POSITION", 1);
  ros::Publisher best_frontier_pub = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird/reference", 1);
  // ros::Publisher chosen_path_pub = nh.advertise<nav_msgs::Odometry>("/pelican/reference", 1);
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
    cout << "Local frontiers array size is " << local.list.size() << endl;
    cout << "Global frontiers array size is " << global.list.size() << endl;
    

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

      if (ext.size() > 0) {
      
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
      } else {
          selectedgfrontier = selectGlobalFrontier(global.list, current.position, current.heading);
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

    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    cout << "\nExecution time: " << duration_total.count() << " ms " << endl;
    visualize(&points_pub, &map_pub, &position_pub, &hits_pub, &local_nodes_pub);
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
