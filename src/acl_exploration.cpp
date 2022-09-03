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
#include <limits> // for numeric_limits
#include <set>
#include <utility> // for pair
#include <algorithm>
#include <iterator>
#include <iostream>
#include <vector>
#include <queue>
 
using namespace std::chrono;
using namespace std;
using namespace std::this_thread;

// Struct

typedef std::list<std::pair<ufo::math::Vector3, double>> old_poses_list_t;

typedef std::vector<std::pair<std::pair<std::pair<ufo::math::Vector3, double>, 
                      std::pair<ufo::math::Vector3, double>>, double>> graph_vector_t;

struct Frontier_main {
  std::list<ufo::math::Vector3> list;
  ufo::math::Vector3 position;
  geometry_msgs::PoseStamped pose;
  double heading;
  nav_msgs::Path path;
};
//Edge class declared early so that,
//it is available for use in Vertex class
class Edge;
 
class Vertex{
public:
  //vertex label
  ufo::math::Vector3 name;
  //Distance value               
  double distance = numeric_limits<double>::infinity();
  //To mark it visited           
  bool visited = false;
  //Edges connected to the vertex, will lead to next vertex              
  list<Edge*> adjacentEdge;
  //distance value updated by which vertex  
  Vertex* predecessor = nullptr;       

  Vertex(ufo::math::Vector3 name){
    this->name = name;
  }
 
  //method to add connect edge with current vertex
  void addAdjacentEdge(Edge* e){
    adjacentEdge.push_back(e);
  }
};
 
//Will return true if v1's distance < v2's distance
//Will be used by priority queue
class Compare{
public:
  bool operator()(Vertex *v1, Vertex *v2){
    return v1->distance < v2->distance? true : false;
  }
};
 
class Edge{
public:
  Vertex *from;       
  Vertex *to;      
  int weight;               

  Edge(Vertex* s, Vertex* t, int w){
    this->from = s;
    this->to = t;
    this->weight = w;
  }
};
 
class DijkstraAlgorithm{
public:
  Vertex *source;
  priority_queue<Vertex*, vector<Vertex*>, Compare> vertexQueue;

  DijkstraAlgorithm(Vertex *s){
    this->source = s;
  }

  //main function to implement Dijkstra's Algorithm
  void computePath(){
    source->distance = 0;          //Starting vertex distance should be 0
    list<Edge*> :: iterator eit;   //Iterator to iterate through edges

    vertexQueue.push(source);

    //run algorithm until queue is empty
    while(!vertexQueue.empty()){     
      //get vertex from the top of the queue 
      //i.e. vertex with least distance
      Vertex* current_vertex = vertexQueue.top();
          
      for(Edge* edge: current_vertex->adjacentEdge){

        Vertex *v = edge->to;

        //Check if vertex is not visited
        if( !v->visited){
          //Update its distance with the lowest value
          if(current_vertex->distance + edge->weight < v->distance){
              v->distance = current_vertex->distance + edge->weight;
              //also update the preceding vertex
              v->predecessor =  current_vertex;     
              vertexQueue.push(v);
          }
        }
      }

      //pop the current vertex from the queue
      vertexQueue.pop(); 
      //mark vertex as visited                     
      current_vertex->visited = true;         
    }
  }

  //method to return shortest path in the form of a list
  list<Vertex*> shortestPath(Vertex *d){
    list<Vertex*> shortPath;

    shortPath.push_front(d);
    while(d->predecessor != nullptr){
      shortPath.push_front(d->predecessor );
      d = d->predecessor ;
    }
    return shortPath;
  }
};


// Variables

int diy;
bool did = false;
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
bool first_gn;
int myindex = 0;
int goaldis;
bool explore_begin = true;
// double heading = 0;

bool frontiers_received = false;
Frontier_main all;
Frontier_main local;
Frontier_main global;
Frontier_main current;
Frontier_main old_poses;
Frontier_main traversable;
// Graph edge;
ufo::math::Vector3 selectedfrontier;
ufo::math::Vector3 selectedgfrontier;
ufo::math::Vector3 nearest_to_global;
double nearest_to_global_id;
geometry_msgs::PoseStamped best_frontier;
nav_msgs::Odometry m_currentPose;
nav_msgs::Odometry m_prevPose;


ufo::map::OccupancyMapColor myMap(RESOLUTION);

// Lists
// std::list<ufo::math::Vector3> extracted;
std::list<ufo::math::Vector3> ext;
std::list<ufo::math::Vector3> accessible;
std::list<ufo::math::Vector3> positionslist;
old_poses_list_t akash;
old_poses_list_t visited_and_global;
graph_vector_t COMPLETE_GRAPH;

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
                                                          + pow(point1.z() - point2.z(), 2)));
  
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
      ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.5, 0.5),
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

  double angle = calculateAngle(front, position, heading);
  double distance = calculateDistance(front, position);

    if (((angle < 40) && (angle > -40)) && distance < 10) {

      return true;

    } else {

      return false;
    }

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
      // if ( (globalfrontier.z() > 3) || (globalfrontier.z() < 0.8)) {
      //     score = score + 1000;
      // } else {
      //     score = score;
      // }

      if (score < myScore) {              
          myScore = score;
          selectg = globalfrontier;
      }
  }
  return selectg;
}

bool next_pub(const geometry_msgs::PoseStamped &best_frontier) {

  ufo::math::Vector3 pos;
  pos.x() = best_frontier.pose.position.x;
  pos.y() = best_frontier.pose.position.y;
  pos.z() = best_frontier.pose.position.z;

  double pub_dis = calculateDistance(pos, current.position);
  if (pub_dis < 2) {
    return true;
  } else {
    return false;
  }

}

std::pair<ufo::math::Vector3, double> visitedToGlobal(old_poses_list_t visited, ufo::math::Vector3 global_fronier,
                                          ufo::map::OccupancyMapColor const& map) {
  double num;
  double mynum = 10000000;
  old_poses_list_t::iterator it;
  std::pair<ufo::math::Vector3, double> cp;
  // std::list<ufo::math::Vector3>::iterator it;
  for (it = visited.begin(); it != visited.end(); it++) {


    // The direction from position to goal
    ufo::math::Vector3 direction = it->first - global_fronier;
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
    ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 1, 1),
            ufo::math::Quaternion(roll, pitch, yaw));


    double m = calculateDistance(it->first, global_fronier);
    num = m;
    if (num < mynum && !isInCollision(map, obb, true, false, false, 0)) {
      mynum = num;
      cp = std::make_pair(it->first, it->second);
      // cp = *it;
    }
  }
  return cp;

}

// create a global graph for navigation

std::vector<std::pair<std::pair<std::pair<ufo::math::Vector3, double>, std::pair<ufo::math::Vector3, double>>, double>> createCompleteGraph(
                          old_poses_list_t oldlist, ufo::map::OccupancyMapColor const& map){
  int k;
  std::vector<std::pair<std::pair<std::pair<ufo::math::Vector3, double>, std::pair<ufo::math::Vector3, double>>, double>> myvec;
  k=0; // k is index of array lines

  old_poses_list_t::iterator it;
  
  for (it = oldlist.begin(); it != std::prev(oldlist.end(), 1); it++) {
    old_poses_list_t::iterator it2;
    for (it2 = std::next(it, 1); it2 != oldlist.end(); it2++) {
      

      ufo::math::Vector3 direction = (it->first) - (it2->first);
      // The center between position and goal
      ufo::math::Vector3 center = (it2->first) + (direction / 2.0);
      // The distance between position and goal
      double distance = direction.norm();
      // Normalize direction
      direction /= distance;

      // Calculate yaw, pitch and roll for oriented bounding box
      double yaw = -atan2(direction[1], direction[0]);
      double pitch = -asin(direction[2]);
      double roll = 0;  // TODO: Fix
      
      // Create an oriented bounding box between position and goal, with a size radius.
      ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.8, 0.8),
              ufo::math::Quaternion(roll, pitch, yaw));

      if (!isInCollision(map, obb, true, false, false, 0) && distance < 10) {
      
        // myvec.insert(myvec.end(), std::make_pair(std::make_pair((it->first), (it2->first)), calculateDistance((it->first), (it2->first))));
        myvec.insert(myvec.end(), std::make_pair(std::make_pair(std::make_pair((it->first), (it2->second)), 
                                                std::make_pair((it2->first), (it2->second))), calculateDistance((it->first), (it2->first))));
      }
    }
  }
return myvec;
}


std::pair<std::pair<std::pair<ufo::math::Vector3, double>, std::pair<ufo::math::Vector3, double>>, double> closestInGraph(graph_vector_t cg) {

    graph_vector_t::iterator it;
    
    double mypp = 100000000;

    std::pair<std::pair<std::pair<ufo::math::Vector3, double>, std::pair<ufo::math::Vector3, double>>, double> close;

    for (it = cg.begin(); it != cg.end(); it++) {

      double pp = calculateDistance((((it)->first).first.first), current.position);

      if (pp < mypp) {

        mypp = pp;
        close = *it;

      }

    }
  
  return close;

}

geometry_msgs::PoseStamped publish_goal(ufo::math::Vector3 selected) {
  geometry_msgs::PoseStamped pub_frontier;
  pub_frontier.pose.position.x = selected.x();
  pub_frontier.pose.position.y = selected.y();
  pub_frontier.pose.position.z = selected.z();
  pub_frontier.pose.orientation.x = 0;
  pub_frontier.pose.orientation.y = 0;
  pub_frontier.pose.orientation.z = atan2((selected.y() - current.position.y()), (selected.x() - current.position.x()));;
  pub_frontier.pose.orientation.w = 0;
  // best_frontier_pub.publish(pub_frontier);
  return pub_frontier;
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

    
    if (calculateDistance(newPoint, oldPoint) > 1.5) {
      m_prevPose = *msg;
      positionslist.push_back(oldPoint);
      myindex = myindex + 1;
      akash.push_back(std::make_pair(oldPoint, myindex));
      visited_and_global.push_back(std::make_pair(oldPoint, myindex));
      
    }


    if (calculateDistance(newPoint, oldPoint) > 1.6) {
      move_flag = true;
      
      
    } else {
      move_flag = false;
    }

    old_poses.list = positionslist;

  }

  current.position = ufo::math::Vector3(position_x, position_y, position_z);
  current.pose.pose.position.x = position_x;
  current.pose.pose.position.y = position_y;
  current.pose.pose.position.z = position_z;
  current.pose.pose.orientation.x = 0;
  current.pose.pose.orientation.y = 0;
  current.pose.pose.orientation.z = 0;
  current.pose.pose.orientation.w = 0;
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
  old_poses_list_t mylist3;
  for (geometry_msgs::Point frontier : msg->points) {

    frontier_point = ufo::math::Vector3(frontier.x, frontier.y, frontier.z);

    // The robot's size (radius)
    double radius = 1;

    // Sphere with center at position and radius radius.
    ufo::geometry::Sphere sphere(frontier_point, radius);

    // Check if the robot is currently in collision with occupied space 
    // at the finest map resolution, depth 0 (2 cm).
    if (!isInCollision(myMap, sphere, true, false, false, 0)) {    

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
  if (local.list.size() != 0) {
  traversable.list = extractUnknowns(myMap, local.list, current.position, true, true, true, 0);
  }

}

// Publish all visualization messages here.

void visualize(ros::Publisher* position_pub, ros::Publisher* points_pub, 
                                                      ros::Publisher* hits_pub, ros::Publisher* local_nodes_pub, ros::Publisher* global_nodes_pub,
                                                              ros::Publisher global_connection_visualization_pub,
                                                                    ros::Publisher global_connection_line_pub){
  visualization_msgs::Marker POSITION_point, HITS_points, local_frontier_nodes, global_frontier_nodes, global_connection_points, global_connection_line_list;

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
    // POSITION_point.color.g = 1.0f;
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


  if(traversable.list.size() != 0){
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
    for(it_comeon_visualizer2 = traversable.list.begin(); it_comeon_visualizer2 != traversable.list.end(); it_comeon_visualizer2++){
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

  if(global.list.size() != 0){
    global_frontier_nodes.header.frame_id = "world";
    global_frontier_nodes.ns = "points";
    global_frontier_nodes.action = visualization_msgs::Marker::ADD;
    global_frontier_nodes.pose.orientation.w = 1.0;
    global_frontier_nodes.id = 0;
    global_frontier_nodes.type = visualization_msgs::Marker::POINTS;
    global_frontier_nodes.scale.x = 0.4;
    global_frontier_nodes.scale.y = 0.4;
    global_frontier_nodes.color.b = 0.9f;
    global_frontier_nodes.color.r = 0.9f;
    global_frontier_nodes.color.g = 0.9f;
    global_frontier_nodes.color.a = 0.9;
    std::list<ufo::math::Vector3>::iterator it_comeon_visualizer5;	
    for(it_comeon_visualizer5 = global.list.begin(); it_comeon_visualizer5 != global.list.end(); it_comeon_visualizer5++){
      geometry_msgs::Point p0;
      p0.x = it_comeon_visualizer5->x();
      p0.y = it_comeon_visualizer5->y();
      p0.z = it_comeon_visualizer5->z();
      global_frontier_nodes.points.push_back(p0);
    }
    global_nodes_pub->publish(global_frontier_nodes);
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

    std::vector<std::pair<std::pair<std::pair<ufo::math::Vector3, double>, 
                                  std::pair<ufo::math::Vector3, double>>, double>> mp = createCompleteGraph(visited_and_global, myMap);
    geometry_msgs::Point p1;
      p1.x = 0;
      p1.y = 0;
      p1.z = 0;
      global_connection_points.points.push_back(p1);
      
        for (auto i = mp.begin(); i != mp.end(); i++) {

        
          // global_connection_line_list.points.push_back(p2);
          geometry_msgs::Point p2;
          p2.x = (((*i).first).first).first.x();
          p2.y = (((*i).first).first).first.y();
          p2.z = (((*i).first).first).first.z();
          global_connection_line_list.points.push_back(p2);
          geometry_msgs::Point p3;
          p3.x = (((*i).first).second).first.x();
          p3.y = (((*i).first).second).first.y();
          p3.z = (((*i).first).second).first.z();
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
  ros::Publisher path_to_goal_pub = nh.advertise<nav_msgs::Path>("/hummingbird/path", 1);
  ros::Publisher global_connection_visualization_pub = nh.advertise<visualization_msgs::Marker>("/points_global_connection", 1);
  ros::Publisher global_connection_line_pub = nh.advertise<visualization_msgs::Marker>("/line_global_connection", 1);
  ros::Publisher hits_pub = nh.advertise<visualization_msgs::Marker>("Traversable_frontiers", 1);
  ros::Publisher local_nodes_pub = nh.advertise<visualization_msgs::Marker>("local_frontier_nodes_vis", 1);
  ros::Publisher global_nodes_pub = nh.advertise<visualization_msgs::Marker>("global_frontier_nodes_vis", 1);
  ros::Subscriber map_sub = nh.subscribe("ufomap_mapping_server_node/map", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("/hummingbird/ground_truth/odometry", 1, odomCallback);
  ros::Subscriber front_sub = nh.subscribe("/frontiers_nodes", 1, frontiersCallback);

  ros::Rate rate(10);


  
  // Dynamic reconfigure
  dynamic_reconfigure::Server<acl_exploration::ServerConfig> server;
  dynamic_reconfigure::Server<acl_exploration::ServerConfig>::CallbackType f;
  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);
  

  
  // Main


  while(ros::ok()){

    // if (frontiers_received) {
    // if (explore_begin) {
    //   ros::Duration(2).sleep();
    //   explore_begin = false;
    // }
    
    high_resolution_clock::time_point start_total = high_resolution_clock::now();

    /*
      main computations to be executed here on

    */

    best_frontier.header.stamp = ros::Time::now();
    best_frontier.header.frame_id = "world";
    current.path.header.stamp = ros::Time::now();
    current.path.header.frame_id = "world";
    
    geometry_msgs::PoseStamped initialize;
    if (firstTime) {

      firstTime = false;
      initialize.pose.position.x = 2;
      initialize.pose.position.y = 0;
      initialize.pose.position.z = 1;
      initialize.pose.orientation.x = 0;
      initialize.pose.orientation.y = 0;
      initialize.pose.orientation.z = 0;
      initialize.pose.orientation.w = 0;
      best_frontier_pub.publish(initialize);
      cout << "Initializing exploration....! \n" << endl; 
      ros::Duration(1).sleep();
      cout << "\nlocal : " << local.list.size();
      cout << "\nGlobal : " << global.list.size();
      cout << "\nTraversable : " << traversable.list.size(); 
    }

    while (next_pub(best_frontier) || next_pub(initialize)) {

      while (traversable.list.size() == 0 && global.list.size() != 0) {
          selectedgfrontier = selectGlobalFrontier(global.list, current.position, current.heading);
          cout << "global frontier X: " << selectedgfrontier.x() << endl;
          cout << "global frontier Y: " << selectedgfrontier.y() << endl;
          cout << "global frontier Z: " << selectedgfrontier.z() << endl;

          for (ufo::math::Vector3 globalF : global.list) {

          ufo::math::Vector3 direction = globalF - current.position;
          ufo::math::Vector3 center = current.position + (direction / 2.0);
          double distance = direction.norm();
          direction /= distance;
          double yaw = -atan2(direction[1], direction[0]);
          double pitch = -asin(direction[2]);
          double roll = 0;  // TODO: Fix
          ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 1, 1),
          ufo::math::Quaternion(roll, pitch, yaw));

          if (!isInCollision(myMap, obb, true, false, false, 0)) {

              best_frontier = publish_goal(globalF);
              best_frontier_pub.publish(best_frontier);
              ROS_INFO("Global Frontier with colission check exist -- Published");
              current.path.poses.push_back(current.pose);
              current.path.poses.push_back(best_frontier);
              path_to_goal_pub.publish(current.path);
              current.path.poses.clear();
          } else {
            did = true;
          }
          }
           if (did) {

              selectedgfrontier = selectGlobalFrontier(global.list, current.position, current.heading);
              if(old_poses.list.size() > 2) {
                std::vector<Vertex *> v;
                std::list<ufo::math::Vector3>::iterator old_list_it;
                for (old_list_it = old_poses.list.begin(); old_list_it != old_poses.list.end(); old_list_it++) {
                    v.push_back(new Vertex {*old_list_it});
                }
                v.push_back(new Vertex {selectedgfrontier});
                std::vector<Edge *> e;
                for (auto node_it = v.begin(); node_it != --v.end(); node_it++) {
                  for (auto node_it2 = std::next(v.begin(), 1); node_it2 != v.end(); node_it2++) {
                    ufo::math::Vector3 direction = ((*node_it)->name) - ((*node_it2)->name);
                    ufo::math::Vector3 center = ((*node_it)->name) + (direction / 2.0);
                    double distance = direction.norm();
                    direction /= distance;
                    double yaw = -atan2(direction[1], direction[0]);
                    double pitch = -asin(direction[2]);
                    double roll = 0;  // TODO: Fix
                    ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.2, 0.2),
                            ufo::math::Quaternion(roll, pitch, yaw));
                    if (!isInCollision(myMap, obb, true, false, false, 0)) {
                      e.push_back(new Edge {(*node_it), (*node_it2), int(calculateDistance((*node_it)->name, (*node_it2)->name)) } );
                      e.push_back(new Edge {(*node_it2), (*node_it), int(calculateDistance((*node_it)->name, (*node_it2)->name)) } );
                    }
                  }
                }
                for (auto nd_it = v.begin(); nd_it != v.end(); nd_it++) {
                  for (auto edge_it = e.begin(); edge_it != e.end(); edge_it++) {
                      if ((((*nd_it)) == ((*edge_it)->from)) || (((*nd_it)) == ((*edge_it)->to))) {
                        (*nd_it)->addAdjacentEdge (*edge_it);
                      }
                  }
                }
                int nup;
                ufo::math::Vector3 nrv;
                nrv = visitedToGlobal(akash, selectedgfrontier, myMap).first;
                for (auto nrt = v.begin(); nrt != v.end(); nrt++) {
                  if ((*nrt)->name == nrv) {
                    nup = std::distance(v.begin(), nrt);
                  }
                }
                DijkstraAlgorithm da((v.front()));
                da.computePath();
                list<Vertex*> result= da.shortestPath(*(--v.end()));
                // result.push_back(new Vertex {selectedgfrontier});
                cout<< "\033[;32mShortest path iterates through vertices:\033[0m\n";
                for(auto vert_it = result.begin(); vert_it != result.end(); vert_it++){
                  cout << "X : " << ((*vert_it)->name).x() << "\n ";
                  cout << "Y : " << ((*vert_it)->name).y() << "\n ";
                  cout << "Z : " << ((*vert_it)->name).z() << "\n ";
                  cout << "--------" << "\n ";
                  geometry_msgs::PoseStamped global_nodes;
                  global_nodes.pose.position.x = ((*vert_it)->name).x();
                  global_nodes.pose.position.y = ((*vert_it)->name).y();
                  global_nodes.pose.position.z = ((*vert_it)->name).z();
                  global_nodes.header.stamp = ros::Time::now();
                  global_nodes.header.frame_id = "world";
                  // current.path.poses.push_back(current.pose);
                  current.path.poses.push_back(global_nodes);
                }
                path_to_goal_pub.publish(current.path);
                current.path.poses.clear();
                first_gn = true;
                for (auto gn_it = result.begin(); gn_it != result.end(); gn_it++) {
                  geometry_msgs::PoseStamped retrace;
                  best_frontier = publish_goal((*gn_it)->name);
                  best_frontier_pub.publish(best_frontier);
                  double wp_dis = abs(sqrt(pow((*gn_it)->name.x() - current.position.x(), 2) 
                                                        + pow((*gn_it)->name.y() - current.position.y(), 2) 
                                                              + pow((*gn_it)->name.z() - current.position.z(), 2)));
                  if(first_gn || (wp_dis < 2)) {
                    best_frontier_pub.publish(best_frontier);
                    ROS_INFO("\nRetracing......");  
                    first_gn = false;
                  }     
                }
              }
          }
        break;
      } 

      while (local.list.size() > 0 && traversable.list.size() != 0) {

            selectedfrontier = selectLocalFrontier(traversable.list, current.position, current.heading);
            best_frontier = publish_goal(selectedfrontier);
            best_frontier_pub.publish(best_frontier);
            ROS_INFO("Local Frontier Published");
            current.path.poses.push_back(current.pose);
            current.path.poses.push_back(best_frontier);
            path_to_goal_pub.publish(current.path);
            current.path.poses.clear();
            traversable.list = extractUnknowns(myMap, local.list, current.position, true, true, true, 0);
            break;

      }

  break;
  } 
  
  // else {
  //       cout << "Exploration complete" << endl;
  //       ros::Duration(2).sleep();
  //     }

    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    
    cout << "\033[;34mExecution time\033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;
    
    visualize(&points_pub, &position_pub, &hits_pub, &local_nodes_pub, &global_nodes_pub, global_connection_visualization_pub,
                                                                    global_connection_line_pub);
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
3) Fix heading deviation based search -- done
4) Active loop closing --TBD

*/
