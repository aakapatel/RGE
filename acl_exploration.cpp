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
  ufo::math::Vector3 something;
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
double heading = 0;

bool frontiers_received;
Frontier_main frontiers;
ufo::math::Vector3 selectedlocalfrontier;

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

ufo::math::Vector3 selectLocalFrontier(std::list<ufo::math::Vector3> accessible, 
                                                ufo::math::Vector3 position, double heading) {

  double cost;
  double mycost = 100000000;
  ufo::math::Vector3 select;
  for (ufo::math::Vector3 localfrontier : accessible) {

    double head = (atan2((localfrontier.y() - position.y()), (localfrontier.x() - position.x())) * (180/3.14159));

    double angle = head - heading;
    double distance = calculateDistance(localfrontier, position);

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

    if (((angle < 40) && (angle > -40)) && distance < 14) {

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




// Publish all visualization messages here.

void visualize(ros::Publisher* position_pub, ros::Publisher* points_pub, ros::Publisher* map_pub, ros::Publisher* hits_pub){
  visualization_msgs::Marker POSITION_point, HITS_points;

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
  tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  heading = yaw * (180/3.14159);
  
  
}

void frontiersCallback(const exploration::Frontiers::ConstPtr& msg) {

  frontiers_received = true;
  ufo::math::Vector3 frontier_point;
  
  std::list<ufo::math::Vector3> mylist;
  for (geometry_msgs::Point frontier : msg->points) {

    frontier_point = ufo::math::Vector3(frontier.x, frontier.y, frontier.z);
    mylist.push_back(frontier_point);
    frontiers.list = mylist;
    // std::cout << "the frontiers list size is: " << frontiers.list.size() << std::endl;

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
  ros::Publisher best_local = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird/reference", 1);
  // ros::Publisher chosen_path_pub = nh.advertise<nav_msgs::Odometry>("/pelican/reference", 1);
  ros::Publisher hits_pub = nh.advertise<visualization_msgs::Marker>("Frontiers", 1);
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

    ufo::math::Vector3 position(position_x, position_y, position_z);

    // Find closest free space within a sphere
    // The sphere's radius
      double radius = 10.2;

    // Sphere with center at position and radius radius.
    ufo::geometry::Sphere sphere(position, radius);

    ext = extractUnknowns(myMap, frontiers.list, position, true, true, true, 0);
    cout << "Accessible frontiers array size is " << ext.size() << endl;

    selectedlocalfrontier = selectLocalFrontier(ext, position, heading);
    geometry_msgs::PoseStamped best_local_frontier;
    double head_ref = atan2((selectedlocalfrontier.y() - position.y()), (selectedlocalfrontier.x() - position.x()));
    best_local_frontier.pose.position.x = selectedlocalfrontier.x();
    best_local_frontier.pose.position.y = selectedlocalfrontier.y();
    best_local_frontier.pose.position.z = selectedlocalfrontier.z();
    best_local_frontier.pose.orientation.x = 0;
    best_local_frontier.pose.orientation.y = 0;
    best_local_frontier.pose.orientation.z = head_ref;
    best_local_frontier.pose.orientation.w = 0;
    best_local.publish(best_local_frontier);
    
    // cout << "Selected local frontier X: " << selectedlocalfrontier.x() << endl;
    // cout << "Selected local frontier Y: " << selectedlocalfrontier.y() << endl;
    // cout << "Selected local frontier Z: " << selectedlocalfrontier.z() << endl;

    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    cout << "\nExecution time: " << duration_total.count() << " ms " << endl;
    visualize(&points_pub, &map_pub, &position_pub, &hits_pub);
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
