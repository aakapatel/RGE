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
#include "std_msgs/String.h"
#include <limits> 
#include <set>
#include <utility> 
#include <algorithm>
#include <iterator>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <climits> 
#include <memory>

using namespace std::chrono;
using namespace std;
using namespace std::this_thread;

struct node{
   public:
      ufo::math::Vector3* point;
      node* myParent = nullptr;
      double distanceToParent = -1;
      std::list<struct node*> myChilds{};
      std::list<struct node*> myPath{};
      std::list<ufo::math::Vector3> myHits{};
      double distanceToGoal;
      node(float x, float y, float z){
         point = new ufo::math::Vector3(x, y, z);
      }
      node(ufo::math::Vector3* givenPoint){
        point = new ufo::math::Vector3(givenPoint->x(), givenPoint->y(), givenPoint->z());
      }
      ufo::math::Vector3* getNodePointer(){
         return point;
      }
      void addChild(node* newChild){
        myChilds.push_back(newChild);
      }
      void addParent(node* newParent){
        myParent = newParent;
      }
      void changeDistanceToGoal(double newDistance){
        distanceToGoal = newDistance;
      }
      void changeDistanceToParent(){
        if(myParent != nullptr and distanceToParent != -1){
          distanceToParent = sqrt(pow(point->x() - myParent->point->x(), 2) + pow(point->y() - myParent->point->y(), 2) + pow(point->z() - myParent->point->z(), 2));
        }
        else{
          distanceToParent = 0;
        }
      }
      double sumDistance(){
        changeDistanceToParent();
        double myDistance = 0;
        if(myParent != nullptr){
          myDistance = myParent->sumDistance();
          changeDistanceToParent();
        }
        if(myParent == nullptr){
          return 0;
        }
        return (myDistance + distanceToParent);
      }
      
      void getPath(std::list<struct node*>* givenPath){
        if(myParent != nullptr){
          myParent->getPath(givenPath);
          if(myParent->myParent != nullptr){
            if(myParent->point->x() != point->x() or myParent->point->y() != point->y() or myParent->point->z() != point->z()){ 
              givenPath->push_back(new node(myParent->point->x(), myParent->point->y(), myParent->point->z()));
            }
          }
        }
        if(myParent == nullptr){
          return;
        }
        return;
      }
      
      void readyForDeletion(){
        delete point;
      }
      
      bool isInCollision(ufo::map::OccupancyMapColor const& map, 
                   ufo::geometry::BoundingVar const& bounding_volume, 
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = 3){
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
};


typedef std::list<std::pair<ufo::math::Vector3, double>> old_poses_list_t;

typedef std::vector<std::pair<std::pair<std::pair<ufo::math::Vector3, double>, 
                      std::pair<ufo::math::Vector3, double>>, double>> graph_vector_t;

class Edge;
 
class Vertex{
public:
  ufo::math::Vector3 name;            
  double distance = numeric_limits<double>::infinity();       
  bool visited = false;            
  list<Edge*> adjacentEdge;
  Vertex* predecessor = nullptr;       
  Vertex(ufo::math::Vector3 name){
    this->name = name;
  }
  void addAdjacentEdge(Edge* e){
    adjacentEdge.push_back(e);
  }
};

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
  void computePath(){
    source->distance = 0;         
    list<Edge*> :: iterator eit;   

    vertexQueue.push(source);
    while(!vertexQueue.empty()){     
      Vertex* current_vertex = vertexQueue.top();
          
      for(Edge* edge: current_vertex->adjacentEdge){
        Vertex *v = edge->to;
        if( !v->visited){
          if(current_vertex->distance + edge->weight < v->distance){
              v->distance = current_vertex->distance + edge->weight;
              v->predecessor =  current_vertex;     
              vertexQueue.push(v);
          }
        }
      }
      vertexQueue.pop();                   
      current_vertex->visited = true;         
    }
  }
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



struct Point {
    Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {};
    bool operator ==(const Point& o) const { return o.x == x && o.y == y && o.z == z; }
    double DistTo(Point & other) const { return(std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2) + std::pow(other.z - z, 2))); };
    double DistDistToHeuristicTo(Point & other) const { return( 0.1*(std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2) + std::pow(other.z - z, 2)))); };
    // double DistToHeuristic(Point & other) const { return(abs(other.x - x) + abs(other.y - y) + abs(other.z - z)); };
    double x, y, z;

    void Print(std::string name) const
    {
        std::cout << name << " x: " << x << " y: " << y << " z: " << z << std::endl;
    }
};

struct Node
{
    Node(Point myposition = Point(), Node * parent = nullptr) : myposition(myposition), parent(parent) {};

    Point myposition;

    double dist_to_start = 0;
    double total_dist = INT_MAX;

    std::shared_ptr<Node> parent;
    std::shared_ptr< std::vector<Node> > connections{ new std::vector<Node> };

    bool operator ==(const Node& o) const { return (o.myposition.x == myposition.x) && (o.myposition.y == myposition.y) && (o.myposition.z == myposition.z);}
    bool operator <(const Node& o) const { return (total_dist < o.total_dist); }

    void AddConnection(Node subnode)
    {
        connections->push_back(subnode);
    }
};

struct Frontier_main {
  bool navigate;
  bool improve;
  std::list<ufo::math::Vector3> list;
  ufo::math::Vector3 position;
  geometry_msgs::PoseStamped pose;
  double heading;
  double x_ref;
  double y_ref;
  double z_ref;
  nav_msgs::Path path;
  std::vector<Node> node_container; 
  node* graph_pointer;
  std::vector<Point> traj;
  ros::Publisher points_pub;
  ros::Publisher chosen_path_visualization_pub;
  ros::Publisher alc_points_visualization_pub;
  ros::Publisher alc_path_visualization_pub;
  ros::Publisher position_pub;
  ros::Publisher map_pub; 
  ros::Publisher hits_pub;
  ros::Publisher local_nodes_pub; 
  ros::Publisher global_nodes_pub;
  ros::Publisher explored_path;
  ros::Publisher path_to_goal_pub;
  ros::Publisher best_frontier_pub;
  geometry_msgs::PoseStamped reference;

};

std::vector<Point> Astar(Point & start_pos, Point & end_pos, std::vector<Node> all_nodes)
{

    Node start_node;
    Node end_node;

    for (Node & node : all_nodes)
    {
        if (node.myposition == end_pos) 
            start_node = node;
        if (node.myposition == start_pos)
            end_node = node;
    }

    std::vector<Point> found_path; // Output path


    std::set<Node> open_list; 
    std::vector<Point> closed_list;

    open_list.emplace(start_node);


    while (!open_list.empty())
    {
        Node current_node = *open_list.begin();

        open_list.erase(open_list.begin());
        closed_list.push_back(current_node.myposition);


        if (current_node.myposition == end_node.myposition)
        {
            Node * parent = &current_node;

            while (parent != nullptr)
            {
                found_path.push_back(parent->myposition);
                parent = parent->parent.get();
            }
            return(found_path);
        }

        for (Node& node : *current_node.connections)
        {
            Node child(node);
            child.parent.reset(new Node(current_node));

            if (std::find_if(closed_list.begin(), closed_list.end(), [&child](Point & pt) { return(pt == child.myposition); }) != closed_list.end())
                continue;


            child.dist_to_start = current_node.dist_to_start + child.myposition.DistTo(current_node.myposition);

            child.total_dist = child.dist_to_start + 0.05* (child.myposition.DistTo(end_node.myposition));

            if (std::find_if(open_list.begin(), open_list.end(), [&child](const Node & node) { return((node == child) && (child.dist_to_start > node.dist_to_start)); }) != open_list.end())
                continue;

            open_list.emplace(child);
        }

    }

    return(found_path);
}




bool start_backtrack = false;


// Variables
int NUMBER_OF_NODES;
int NUMBER_OF_GOALS;
int NUMBER_OF_ITTERATIONS;
int NMPC_POINTS;
int itterations;
int advance_index = 0;
float DISTANCE_BETWEEN_NODES; 
float DISTANCE_BETWEEN_GOALS;
float MINIMUM_DISTANCE_TO_GOAL;
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
float lowest_x;
float lowest_y;
float lowest_z;
float highest_x;
float highest_y;
float highest_z;
float averageInfo = 50;
float initialGoalInfo = 0.0;
bool RUN_BY_NODES;
bool map_received = false;
bool RRT_created = false;
bool GOALS_generated = false;
bool fetched_path = false;
bool newPath = false;
bool allowNewPath = true;
bool recoveryUnderway = false;
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
double NEXT_PUB_DISTANCE;
double LOCAL_RANGE;
double RADIOUS;
double RESOLUTION;
double roll = 0;
double pitch = 0;
double yaw = 0;
double totalCost = std::numeric_limits<float>::max();
double totalDistance = -1;
node* goalNode = nullptr;
node* reserveGoalNode = nullptr;
node* currentTarget;
double md;
double head_ref;
bool firstTime = true;
bool start_flag = true;
bool move_flag = false;
bool first_gn;
int myindex = 0;
int goaldis;
bool explore_begin = true;
bool travers;
bool go_global;
bool frontiers_received;

ufo::math::Vector3 selectedfrontier;
ufo::math::Vector3 selectedgfrontier;
ufo::math::Vector3 nearest_to_global;
double nearest_to_global_id;
nav_msgs::Odometry m_currentPose;
nav_msgs::Odometry m_prevPose;


ufo::map::OccupancyMapColor myMap(RESOLUTION);
std::list<ufo::math::Vector3> ext;
std::list<ufo::math::Vector3> accessible;
std::list<ufo::math::Vector3> positionslist;
graph_vector_t COMPLETE_GRAPH;

std::list<struct node*> Global_Graph{};
std::list<struct node*> CHOSEN_PATH{};
std::list<double> CHOSEN_PATH_VREF{};
std::list<double>::iterator vref_itterator;
std::list<struct node*> ALL_PATH{};
std::list<struct node*> myGoals{};
std::list<struct node*> myReserveGoals{};
std::list<ufo::math::Vector3> hits{};
std::list<node*> VISITED_POINTS{};
std::list<node*> ALC_POINTS{};
std::list<node*>::iterator path_itterator;

Frontier_main all;
Frontier_main local;
Frontier_main global;
Frontier_main current;
Frontier_main old_poses;
Frontier_main traversable;
Frontier_main pub;
Frontier_main max_information;
Frontier_main global_frontier;
Frontier_main planning;
Frontier_main planned;
Frontier_main published;
Frontier_main starting;
Frontier_main ending;
Frontier_main home;
Frontier_main explore;

// home.navigate = false;

//--------------------------------- FUNCTIONS ----------------------------------------------//


// TUNE GRAPH GENERATION. LOCAL AND GLOBAL BOUNDING SPACE CAN BE CONFIGURED HERE.

void planningSpace(float range, ufo::map::OccupancyMapColor const& map, bool occupied_space, bool free_space, bool unknown_space, float given_x, 
                                      float given_y, float given_z, ufo::map::DepthType min_depth = 3){
  
  highest_x = -9999;
  highest_y = -9999;
  highest_z = -9999;
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();
  ufo::math::Vector3 minPoint(given_x - 1 * range, given_y - 1 * range, given_z - 1 * range);
  ufo::math::Vector3 maxPoint(given_x + 1 * range, given_y + 1 * range, given_z + 1 * range);
  ufo::geometry::AABB aabb(minPoint, maxPoint);
  for (auto it = map.beginLeaves(aabb, occupied_space, free_space, unknown_space, false, min_depth), it_end = map.endLeaves(); it != it_end; ++it) {
    if(it.getX() > highest_x){
      highest_x = it.getX();
    }if(it.getX() < lowest_x){
      lowest_x = it.getX();
    }
    if(it.getY() > highest_y){
      highest_y = it.getY();
    }if(it.getY() < lowest_y){
      lowest_y = it.getY();
    }
    if(it.getZ() > highest_z){
      highest_z = it.getZ();
    }if(it.getZ() < lowest_z){
      lowest_z = it.getZ();
    }
  }
  SCALER_X = highest_x - lowest_x;
  SCALER_Y = highest_y - lowest_y;
  SCALER_Z = highest_z - lowest_z;
}

bool isInCollision(ufo::map::OccupancyMapColor const& map, 
                   ufo::geometry::BoundingVar const& bounding_volume, 
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = 3)
{
  for (auto it = map.beginLeaves(bounding_volume, occupied_space, 
                                 free_space, unknown_space, false, min_depth), 
        it_end = map.endLeaves(); it != it_end; ++it) {
    return true;
  }
  return false;
}

void buildGraph(float given_x, float given_y, float given_z, int max_iterations) {
  for(std::list<node*>::iterator it_clear_helper = Global_Graph.begin(); it_clear_helper != --Global_Graph.end(); it_clear_helper++){
    (*it_clear_helper)->readyForDeletion();
    delete(*it_clear_helper);
  }
  
  Global_Graph.clear();
  std::cout << "\nBuilding graph... " << std::endl;
  node* origin = new node(given_x, given_y, given_z);
  origin->addParent(nullptr);
  Global_Graph.push_back(origin);
  srand(time(0));
  itterations = 0;
  while(itterations < max_iterations) {
    // Generate a random point
    float x = lowest_x + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_X;
    float y = lowest_y + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_X;
    float z = lowest_z + abs(1024 * rand () / (RAND_MAX + 1.0)) * SCALER_X;
    ufo::math::Vector3 random_point(x, y, z);
    ufo::geometry::Sphere point_sphere(random_point, RADIOUS);
    if(!isInCollision(myMap, point_sphere, true, false, true, 0) and isInCollision(myMap, point_sphere, false, true, false, 0)){
      float distance = std::numeric_limits<float>::max();
      node* parent;
      std::list<node*>::iterator it_node;
      for(it_node = Global_Graph.begin(); it_node != Global_Graph.end(); it_node++){
        ufo::math::Vector3 direction = random_point - *((*it_node)->point);
        double new_distance = abs(direction.norm());
        if(new_distance < distance){
          distance = new_distance;
          parent = *it_node;
        }
      };
      ufo::math::Vector3 start_point(parent->point->x(), parent->point->y(), parent->point->z());
      ufo::geometry::LineSegment myLine(random_point, start_point);
      if(!isInCollision(myMap, myLine, true, false, true, 0)) {
          node* new_node = new node(x, y, z);
          new_node->addParent(parent);
          parent->addChild(new_node);
          Global_Graph.push_back(new_node);
      }
    };
    itterations++;
  };
  std::cout << "\nGraph built successfully with " << Global_Graph.size() << " nodes\n" << std::endl;

};

double calculateDistance(ufo::math::Vector3 point1, ufo::math::Vector3 point2) {
  double dist = abs(sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) 
                                                          + pow(point1.z() - point2.z(), 2)));
  return dist;
}

void fetchTraversable(bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = 0)
{
  std::list<ufo::math::Vector3> extracted;
    for (ufo::math::Vector3 goal : max_information.list) {
      ufo::math::Vector3 direction = goal - current.position;
      ufo::math::Vector3 center = current.position + (direction / 2.0);
      double distance = direction.norm();
      direction /= distance;
      double yaw = -atan2(direction[1], direction[0]);
      double pitch = -asin(direction[2]);
      double roll = 0;  // TODO: Fix
      ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.5, 0.5),
              ufo::math::Quaternion(roll, pitch, yaw));
      if (!isInCollision(myMap, obb, true, false, false, 0) && calculateDistance(goal, current.position) > 3) {
        extracted.push_back(goal);
      }      
    }
  traversable.list = extracted;
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
    if (((angle < 40) && (angle > -40)) && distance < LOCAL_RANGE) {
      return true;
    } else {
      return false;
    }
}

ufo::math::Vector3 informativeFrontier () {
  ufo::math::Vector3 most_info_frontier;
  double radius = 0.1;
  for (ufo::math::Vector3 f_point : local.list) {

    ufo::geometry::Sphere sphere(f_point, radius);
    if (isInCollision(myMap, sphere, false, true, false, 0) && !isInCollision(myMap, sphere, true, false, true, 0)) { 
      radius = radius + 0.1;
      most_info_frontier = f_point;
    }

  }
  return most_info_frontier;
}

ufo::math::Vector3 selectLocalFrontier(std::list<ufo::math::Vector3> accessible, 
                                                ufo::math::Vector3 position, double heading) {
  double cost;
  double mycost = 100000000;
  ufo::math::Vector3 select;
  for (ufo::math::Vector3 localfrontier : accessible) {
    double distance = calculateDistance(localfrontier, position);
    double angle = calculateAngle(localfrontier, position, heading);
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
  return select;
}

node* selectGlobalFrontier(std::list<ufo::math::Vector3> global) {
  double score;
  double myScore = 100000000;
  ufo::math::Vector3 selectg;

    for (ufo::math::Vector3 globalfrontier : global) {
        score = calculateDistance(globalfrontier, current.position);
        if (score < myScore) {              
            myScore = score;
            selectg = globalfrontier;
        }
    }
  double pen;
  double mypen = 10000000000000;
  node* my_nod;
  for (auto rit = Global_Graph.begin(); rit != Global_Graph.end(); rit++) {

    double nearest_dis = calculateDistance(*((*rit)->point), selectg);
    pen = nearest_dis;
    if (pen < mypen) {
      mypen = pen;
      my_nod = *rit;
    }

  }
  return my_nod;
}

std::list<node*> selectGlobalPath(std::list<ufo::math::Vector3> global) {
  double score1;
  double myScore1 = 100000000;
  ufo::math::Vector3 selectg1;
  std::list<node*> alc_candidates;
  std::list<ufo::math::Vector3> successful_ray;

  for (ufo::math::Vector3 globalfrontier1 : global) {
      score1 = calculateDistance(globalfrontier1, current.position);
      if (score1 < myScore1) {              
          myScore1 = score1;
          selectg1 = globalfrontier1;
      }
  }

  for (auto old_poses_it = old_poses.list.begin(); old_poses_it != old_poses.list.end(); old_poses_it++) {

      ufo::math::Vector3 start_pt((*old_poses_it).x(), (*old_poses_it).y(), (*old_poses_it).z());
      ufo::geometry::LineSegment myLine(selectg1, start_pt);
      if(!isInCollision(myMap, myLine, true, false, true, 0)) {
        successful_ray.push_back(*old_poses_it);
      }
  }

  for (auto tree_it = Global_Graph.begin(); tree_it != Global_Graph.end(); tree_it++) {

    for (auto ray_it = successful_ray.begin(); ray_it != successful_ray.end(); ray_it++) {

      double d = calculateDistance(*ray_it, *((*tree_it)->point));

      if (d < 0.2) {
        alc_candidates.push_back(*tree_it);
      }

    }

  }

return alc_candidates;  

}

bool next_pub() {

  ufo::math::Vector3 pos;
  pos.x() = published.reference.pose.position.x;
  pos.y() = published.reference.pose.position.y;
  pos.z() = published.reference.pose.position.z;
  double pub_dis = calculateDistance(pos, current.position);
  if (pub_dis < NEXT_PUB_DISTANCE) {
    return true;
  } else {
    return false;
  }

}

// create a global graph for navigation

std::vector<std::pair<std::pair<std::pair<ufo::math::Vector3, double>, std::pair<ufo::math::Vector3, double>>, double>> createCompleteGraph(
                          old_poses_list_t oldlist, ufo::map::OccupancyMapColor const& map){

  std::vector<std::pair<std::pair<std::pair<ufo::math::Vector3, double>, std::pair<ufo::math::Vector3, double>>, double>> myvec;
  old_poses_list_t::iterator it;
  for (it = oldlist.begin(); it != std::prev(oldlist.end(), 1); it++) {
    old_poses_list_t::iterator it2;
    for (it2 = std::next(it, 1); it2 != oldlist.end(); it2++) {
      ufo::math::Vector3 direction = (it->first) - (it2->first);
      ufo::math::Vector3 center = (it2->first) + (direction / 2.0);
      double distance = direction.norm();
      direction /= distance;
      double yaw = -atan2(direction[1], direction[0]);
      double pitch = -asin(direction[2]);
      double roll = 0;  // TODO: Fix
      ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 1, 1),
              ufo::math::Quaternion(roll, pitch, yaw));
      if (!isInCollision(map, obb, true, false, false, 0)) {
        myvec.insert(myvec.end(), std::make_pair(std::make_pair(std::make_pair((it->first), (it2->second)), 
                                                std::make_pair((it2->first), (it2->second))), calculateDistance((it->first), (it2->first))));
      }
    }
  }
return myvec;
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
  return pub_frontier;
}
//--------------------------------------------------  CALL BACK FUNCTIONS --------------------------------------------//
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
	RUN_BY_NODES = config.Run_by_nodes;
	NUMBER_OF_NODES = config.Number_of_nodes;
	NUMBER_OF_GOALS = config.Number_of_goals;
	DISTANCE_BETWEEN_NODES = config.Distance_between_nodes;
	DISTANCE_BETWEEN_GOALS = config.Distance_between_goals;
	MINIMUM_DISTANCE_TO_GOAL = config.Minimum_distance_to_goal;
	RADIOUS = config.Radious;
	SCALER_AABB = config.Bounding_box_radious;
	SCALER_DISTANCE = config.Scaler_distance_cost;
	SCALER_INFORMATION_GAIN = config.Scaler_information_gain;
	SCALER_ACTUATION = config.Scaler_actuation_cost;
	SENSOR_RANGE = config.Sensor_range;
	SENSOR_MIN = config.Sensor_min;
	SENSOR_HORIZONTAL = config.Sensor_angle_horizontal;
	SENSOR_VERTICAL = config.Sensor_angle_vertical;
	NEXT_PATH_DISTANCE = config.Next_path_distance;
	NEXT_POINT_DISTANCE = config.Next_point_distance;
	RESOLUTION = config.Resolution;
  NEXT_PUB_DISTANCE = config.next_pub_distance;
  LOCAL_RANGE = config.local_range;

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
    if (calculateDistance(newPoint, oldPoint) > 0.1) {
      m_prevPose = *msg;
      positionslist.push_back(oldPoint);
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
  std::list<ufo::math::Vector3> seg1;
  std::list<ufo::math::Vector3> seg2;
  std::list<ufo::math::Vector3> seg3;
  std::list<ufo::math::Vector3> seg4;
  std::list<ufo::math::Vector3> seg5;
  std::list<ufo::math::Vector3> seg6;
  std::list<ufo::math::Vector3> seg7;
  std::list<ufo::math::Vector3> seg8;
  std::list<ufo::math::Vector3> myseg;
  std::list<ufo::math::Vector3> mylist;
  std::list<ufo::math::Vector3> mylist1;
  std::list<ufo::math::Vector3> mylist2;
  for (geometry_msgs::Point frontier : msg->points) {
    frontier_point = ufo::math::Vector3(frontier.x, frontier.y, frontier.z);
    double radius = 0.3;
    ufo::geometry::Sphere sphere(frontier_point, radius);
    if (!isInCollision(myMap, sphere, true, false, false, 0)) {    
    mylist.push_back(frontier_point);
    all.list = mylist;
    if (isLocal(frontier_point, current.position, current.heading)) {
      mylist1.push_back(frontier_point);
      local.list = mylist1;
    } else {
      mylist2.push_back(frontier_point);
      global.list = mylist2;
    }
    }
  }

  for (ufo::math::Vector3 localnode : local.list) {

    double alpha = calculateAngle(localnode, current.position, current.heading);

    if ((alpha < -30) && (alpha > -40)) {
        seg1.push_back(localnode);
    }
    if ((alpha < -20) && (alpha > -30)) {       
        seg2.push_back(localnode);
    }
    if ((alpha < -10) && (alpha > -20)) { 
        seg3.push_back(localnode); 
    }
    if ((alpha < 0) && (alpha > -10)) {       
        seg4.push_back(localnode);       
    }
    if ((alpha < 10) && (alpha > 0)) {       
        seg5.push_back(localnode);       
    }
    if ((alpha < 20) && (alpha > 10)) {      
        seg6.push_back(localnode);      
    }
    if ((alpha < 30) && (alpha > 20)) {      
        seg7.push_back(localnode);     
    }
    if ((alpha < 40) && (alpha > 30)) {      
        seg8.push_back(localnode);      
    }

    if (seg1.size() > myseg.size()) {
        myseg = seg1;
    }
    if (seg2.size() > myseg.size()) {
        myseg = seg2;
    }
    if (seg3.size() > myseg.size()) {
        myseg = seg3;
    }
    if (seg4.size() > myseg.size()) {
        myseg = seg4;
    }
    if (seg5.size() > myseg.size()) {
        myseg = seg5;
    }
    if (seg6.size() > myseg.size()) {
        myseg = seg6;
    }
    if (seg7.size() > myseg.size()) {
        myseg = seg7;
    }
    if (seg8.size() > myseg.size()) {
        myseg = seg8;
    }

    max_information.list = myseg;

  }

  fetchTraversable(true, false, false, 0);

  if (!traversable.list.empty()) {
    local.navigate = true;
  } else {
    local.navigate = false;
  }

  if (traversable.list.empty() and !all.list.empty()) {
    global.navigate = true;
  } else {
    global.navigate = false;
  }

}

void exploreLocally() {

  // ufo::math::Vector3 traversable_goal = selectLocalFrontier(traversable.list, current.position, current.heading);
  ufo::math::Vector3 traversable_goal = informativeFrontier();
  if (traversable_goal.x() == 0 and traversable_goal.y() == 0) {
    traversable_goal = selectLocalFrontier(traversable.list, current.position, current.heading);
  }
  published.reference = publish_goal(traversable_goal);
  current.path.poses.push_back(current.pose);
  current.path.poses.push_back(published.reference);
  pub.path_to_goal_pub.publish(current.path);
  current.path.poses.clear();
  ROS_INFO("Exploring Locally...");

}

void fetchGlobalFrontier () {

  double globalscore;
  double myglobalscore = 100000000;
  ufo::math::Vector3 gf;

  for (ufo::math::Vector3 globalfrontier : global.list) {
      globalscore = calculateDistance(globalfrontier, current.position);
      if (globalscore < myglobalscore) {              
          myglobalscore = globalscore;
          gf = globalfrontier;
      }
  }
  global_frontier.position = gf;

}

void buildPlanningGraph (bool improvement, int reit) {

  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  int min = 20000;
  int max_it;
  float original_planning_range = 30.0;
  float planning_range;
  if (improvement) {
    max_it = min + reit*min;
    planning_range = original_planning_range + (reit*original_planning_range);
  } else {
    max_it = min;
    planning_range = 30.0;
  }

  planningSpace(planning_range, myMap, false, true, false, current.position.x(), current.position.y(), current.position.z(), 0);
  buildGraph(current.position.x(), current.position.y(), current.position.z(), max_it);
  
  planning.graph_pointer = selectGlobalFrontier(all.list);

  ALC_POINTS = selectGlobalPath(all.list);
  
  std::vector<Node> my_nodes;
  
  planning.node_container.clear();
  my_nodes.clear();

  Node sample_nd;
  Node sample_nd2;
  if (!Global_Graph.empty()) {
    
    my_nodes.clear();
    
    if (!home.navigate) {

      for (auto insert_helper = Global_Graph.begin(); insert_helper != Global_Graph.end(); insert_helper++) {

        my_nodes.push_back( { Point{ (((*insert_helper)->point)->x()), (((*insert_helper)->point)->y()), (((*insert_helper)->point)->z()) } } );

      }
    }

  }

  if (home.navigate) {

    my_nodes.clear();
    for (auto insert_helper = old_poses.list.begin(); insert_helper != old_poses.list.end(); insert_helper++) {

      my_nodes.push_back( { Point{ insert_helper->x(), insert_helper->y(), insert_helper->z() } } );

    }

  }
  bool pass_once = true;
  for (auto graph_it = my_nodes.begin(); graph_it != --my_nodes.end(); graph_it++) {
    
    ufo::math::Vector3 d1((*graph_it).myposition.x, (*graph_it).myposition.y, (*graph_it).myposition.z);
    

    for (auto graph_it2 = graph_it; graph_it2 != my_nodes.end(); graph_it2++) {
      
      ufo::math::Vector3 d2((*graph_it2).myposition.x, (*graph_it2).myposition.y, (*graph_it2).myposition.z);

      ufo::math::Vector3 direction = d2 - d1;
      ufo::math::Vector3 center = d1 + (direction / 2.0);
      double distance = direction.norm();
      direction /= distance;
      double yaw = -atan2(direction[1], direction[0]);
      double pitch = -asin(direction[2]);
      double roll = 0;  // TODO: Fix
      ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.3, 0.3),
              ufo::math::Quaternion(roll, pitch, yaw));
      double rang = calculateDistance(d1, d2);

      if (rang < 2.5) {

        if (!isInCollision(myMap, obb, true, false, true, 0)) {

          // pass_once = false;
          // sample_nd = *graph_it;
          // sample_nd2 = *graph_it2;
          // (sample_nd).AddConnection(sample_nd2);
          // (sample_nd2).AddConnection(sample_nd);
          (*graph_it).AddConnection(*graph_it2);
          (*graph_it2).AddConnection(*graph_it);
          planning.node_container.push_back(*graph_it);
          planning.node_container.push_back(*graph_it2);
        }
      
      }


    }

  }

  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  cout << "\n\033[;34m Making the planning graph takes... \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;

}

void findPath() {

  high_resolution_clock::time_point start_total = high_resolution_clock::now();

  if (!home.navigate) {

    double close_dis;
    double end_dis;
    double my_end_dis = 100000000;
    double my_close_dis = 100000000;

    for (auto closest_it = planning.node_container.begin(); closest_it != planning.node_container.end(); closest_it++) {
      
      close_dis = std::sqrt(std::pow((*closest_it).myposition.x - current.position.x(), 2) + 
                            std::pow((*closest_it).myposition.y - current.position.y(), 2) +
                            std::pow((*closest_it).myposition.z - current.position.z(), 2));

      end_dis = std::sqrt(std::pow((*closest_it).myposition.x - ((planning.graph_pointer)->point)->x(), 2) + 
                            std::pow((*closest_it).myposition.y - ((planning.graph_pointer)->point)->y(), 2) +
                            std::pow((*closest_it).myposition.z - ((planning.graph_pointer)->point)->z(), 2));
                            
      if (close_dis < my_close_dis) {
        my_close_dis = close_dis;
        starting.x_ref = ((*closest_it).myposition.x);
        starting.y_ref = ((*closest_it).myposition.y);
        starting.z_ref = ((*closest_it).myposition.z);
      }

      if (end_dis < my_end_dis) {
        my_end_dis = end_dis;
        ending.x_ref = ((*closest_it).myposition.x);
        ending.y_ref = ((*closest_it).myposition.y);
        ending.z_ref = ((*closest_it).myposition.z);
      }

    }

    Point start_pos { starting.x_ref, starting.y_ref, starting.z_ref };
    Point end_pos { ending.x_ref, ending.y_ref, ending.z_ref };

    cout << "\nPlanning container has " << planning.node_container.size() << " nodes" << endl;

    planned.traj = Astar(start_pos, end_pos, planning.node_container);

  } else if (home.navigate) {

    cout << "\nHoming Graph has " << planning.node_container.size() << " nodes" << endl;

    Point start_pos_h { ((planning.node_container.back()).myposition.x), ((planning.node_container.back()).myposition.y), ((planning.node_container.back()).myposition.z) };
    Point end_pos_h { ((planning.node_container.front()).myposition.x), ((planning.node_container.front()).myposition.y), ((planning.node_container.front()).myposition.z) };

    planned.traj = Astar(start_pos_h, end_pos_h, planning.node_container);

  }


  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  cout << "\n\033[;34m Path finding takes... \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;

}

void globalReposition () {

  fetchGlobalFrontier();
  ufo::math::Vector3 direction = global_frontier.position - current.position;
  ufo::math::Vector3 center = current.position + (direction / 2.0);
  double distance = direction.norm();
  direction /= distance;
  double yaw = -atan2(direction[1], direction[0]);
  double pitch = -asin(direction[2]);
  double roll = 0;  // TODO: Fix
  ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.5, 0.5),
          ufo::math::Quaternion(roll, pitch, yaw));
  if (!isInCollision(myMap, obb, true, false, true, 0)) {

    published.reference = publish_goal(global_frontier.position);
    current.path.poses.push_back(current.pose);
    current.path.poses.push_back(published.reference);
    pub.path_to_goal_pub.publish(current.path);
    ROS_INFO("Navigating to Traversable Global Node...");
    current.path.poses.clear();

  } else {
    
    high_resolution_clock::time_point start_total = high_resolution_clock::now();
    int reiterate = 1;
    buildPlanningGraph(planning.improve, reiterate);
    findPath();

    label: 
    if (planned.traj.empty()) {
      reiterate = reiterate + 1;
      std::cout << "\nDid not find a good path... requesting improvement.... \n";
      planning.improve = true;
      buildPlanningGraph(planning.improve, reiterate);
      findPath();
      goto label;

    }

    if (!planned.traj.empty()) {

      high_resolution_clock::time_point stop_total = high_resolution_clock::now();
      auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
      cout << "\n\033[;34mGlobal planning took \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;
      
      if (planned.traj.size() > 1) {
      
        cout<< "\033[;32mPlanned path is composed of these nodes: \033[0m\n ";

        for (auto vert_it = std::next(planned.traj.begin(),0); vert_it != planned.traj.end(); vert_it++) {
          cout << "X : " << ((*vert_it)).x << "\n ";
          cout << "Y : " << ((*vert_it)).y << "\n ";
          cout << "Z : " << ((*vert_it)).z << "\n ";
          cout << "--------" << "\n ";
          geometry_msgs::PoseStamped global_nodes;
          global_nodes.pose.position.x = ((*vert_it)).x;
          global_nodes.pose.position.y = ((*vert_it)).y;
          global_nodes.pose.position.z = ((*vert_it)).z;
          global_nodes.header.stamp = ros::Time::now();
          global_nodes.header.frame_id = "world";
          current.path.poses.push_back(global_nodes);

        }

      } else {

        geometry_msgs::PoseStamped global_node;
        global_node.pose.position.x = ((*(planned.traj.begin()))).x;
        global_node.pose.position.y = ((*(planned.traj.begin()))).y;
        global_node.pose.position.z = ((*(planned.traj.begin()))).z;
        global_node.header.stamp = ros::Time::now();
        global_node.header.frame_id = "world";
        current.path.poses.push_back(global_node);

      }

      pub.path_to_goal_pub.publish(current.path);
      current.path.poses.clear();
      ufo::math::Vector3 lastpub(planned.traj.back().x, planned.traj.back().y, planned.traj.back().z);
      published.reference = publish_goal(lastpub); 

    }

  }

  planned.traj.clear();

}

void homingCallback(const std_msgs::String::ConstPtr& base) {

  home.navigate = true;
  explore.navigate = false;

}

void exploreCallback(const std_msgs::String::ConstPtr& base) {

  explore.navigate = true;
  home.navigate = false;

}

void return_to_base_planning() {

  double homing_cost;
  double myhoming_cost = 10000000000000;
  for (auto rit_h = Global_Graph.begin(); rit_h != Global_Graph.end(); rit_h++) {
    double home_dis = calculateDistance(*((*rit_h)->point), home.position);
    homing_cost = home_dis;
    if (homing_cost < myhoming_cost) {
      myhoming_cost = homing_cost;
      home.graph_pointer = *rit_h;
    }
  }

  high_resolution_clock::time_point start_total = high_resolution_clock::now();

  int reiterate = 1;
  buildPlanningGraph(planning.improve, reiterate);
  findPath();
  
  label: 
  if (planned.traj.empty()) {
    reiterate = reiterate + 1;
    std::cout << "\nDid not find a good path... requesting improvement.... \n";
    planning.improve = true;
    buildPlanningGraph(planning.improve, reiterate);
    findPath();
    goto label;

  }
  

  if (!planned.traj.empty()) {
    
    planning.improve = false;
    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    cout << "\n\033[;34mReturn to base planning took \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;
    
    if (planned.traj.size() > 1) {
    
      cout<< "\033[;32mPlanned path to Base is composed of these nodes: \033[0m\n ";

      for (auto vert_it = std::next(planned.traj.begin(),0); vert_it != planned.traj.end(); vert_it++) {
        cout << "X : " << ((*vert_it)).x << "\n ";
        cout << "Y : " << ((*vert_it)).y << "\n ";
        cout << "Z : " << ((*vert_it)).z << "\n ";
        cout << "--------" << "\n ";
        geometry_msgs::PoseStamped global_nodes;
        global_nodes.pose.position.x = ((*vert_it)).x;
        global_nodes.pose.position.y = ((*vert_it)).y;
        global_nodes.pose.position.z = ((*vert_it)).z;
        global_nodes.header.stamp = ros::Time::now();
        global_nodes.header.frame_id = "world";
        current.path.poses.push_back(global_nodes);

      }

    } else {

      geometry_msgs::PoseStamped global_node;
      global_node.pose.position.x = ((*(planned.traj.begin()))).x;
      global_node.pose.position.y = ((*(planned.traj.begin()))).y;
      global_node.pose.position.z = ((*(planned.traj.begin()))).z;
      global_node.header.stamp = ros::Time::now();
      global_node.header.frame_id = "world";
      current.path.poses.push_back(global_node);

    }

    pub.path_to_goal_pub.publish(current.path);
    current.path.poses.clear();
    ufo::math::Vector3 lastpub(planned.traj.back().x, planned.traj.back().y, planned.traj.back().z);
    published.reference = publish_goal(lastpub); 

  }

  planned.traj.clear();

}

nav_msgs::Path backTrack() {

nav_msgs::Path retrace;
retrace.poses.push_back(current.pose);
  if (start_backtrack) {

      std::list<ufo::math::Vector3>::iterator backtrack_it;
      for (backtrack_it = old_poses.list.end(); backtrack_it != old_poses.list.begin(); backtrack_it--) {

      geometry_msgs::PoseStamped global_nodes2;
      global_nodes2.pose.position.x = backtrack_it->x();
      global_nodes2.pose.position.y = backtrack_it->y();
      global_nodes2.pose.position.z = backtrack_it->z();
      global_nodes2.header.stamp = ros::Time::now();
      global_nodes2.header.frame_id = "world";
      retrace.poses.push_back(global_nodes2);

    }
  }

  return retrace;
}

void visualize(){
  visualization_msgs::Marker GRAPH_nodes, GRAPH_line_list, CHOSEN_PATH_points, CHOSEN_PATH_line_list, 
                            ALC_points, ALC_line_list, POSITION_point, TRAVERSABLE_points, 
                                                local_frontier_nodes, global_frontier_nodes;

  if (!Global_Graph.empty()) {
    GRAPH_nodes.header.frame_id = GRAPH_line_list.header.frame_id = "world";
    GRAPH_nodes.ns = "points";
    GRAPH_nodes.action = visualization_msgs::Marker::ADD;
    GRAPH_nodes.pose.orientation.w = 1.0;
    GRAPH_nodes.id = 0;
    GRAPH_line_list.id = 1;
    GRAPH_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    GRAPH_line_list.type = visualization_msgs::Marker::LINE_LIST;
    GRAPH_nodes.scale.x = 0.3;
    GRAPH_line_list.scale.x = 0.08;
    GRAPH_nodes.color.r = 0.8f;
    GRAPH_nodes.color.g = 0.8f;
    // GRAPH_nodes.color.b = 0.8f;
    GRAPH_nodes.color.a = 1.0;
    GRAPH_line_list.color.b = 0.8;
    GRAPH_line_list.color.a = 1.0;
    std::list<node*>::iterator it_comeon_visualizer;
    for(it_comeon_visualizer = Global_Graph.begin(); it_comeon_visualizer != Global_Graph.end(); it_comeon_visualizer++){
      geometry_msgs::Point p;
      p.x = (*it_comeon_visualizer)->point->x();
      p.y = (*it_comeon_visualizer)->point->y();
      p.z = (*it_comeon_visualizer)->point->z();
      GRAPH_nodes.points.push_back(p);
      if((*it_comeon_visualizer)->myParent != nullptr){
        GRAPH_line_list.points.push_back(p);
        p.x = (*it_comeon_visualizer)->myParent->point->x();
        p.y = (*it_comeon_visualizer)->myParent->point->y();
        p.z = (*it_comeon_visualizer)->myParent->point->z();
        GRAPH_line_list.points.push_back(p);
      }
    }
    pub.points_pub.publish(GRAPH_nodes);
    pub.points_pub.publish(GRAPH_line_list);
  }

    CHOSEN_PATH_points.header.frame_id = CHOSEN_PATH_line_list.header.frame_id = "world";
    CHOSEN_PATH_points.ns = "points";
    CHOSEN_PATH_points.action = visualization_msgs::Marker::ADD;
    CHOSEN_PATH_points.pose.orientation.w = 1.0;
    CHOSEN_PATH_points.id = 0;
    CHOSEN_PATH_line_list.id = 1;
    CHOSEN_PATH_points.type = visualization_msgs::Marker::POINTS;
    CHOSEN_PATH_line_list.type = visualization_msgs::Marker::LINE_LIST;
    CHOSEN_PATH_points.scale.x = 0.2;
    CHOSEN_PATH_points.scale.y = 0.2;
    CHOSEN_PATH_line_list.scale.x = 0.1;
    CHOSEN_PATH_points.color.b = 1.0f;
    CHOSEN_PATH_points.color.a = 1.0;
    CHOSEN_PATH_line_list.color.b = 1.0;
    CHOSEN_PATH_line_list.color.a = 1.0;
    std::list<node*>::iterator it_comeon_visualizer8;
    for(it_comeon_visualizer8 = CHOSEN_PATH.begin(); it_comeon_visualizer8 != CHOSEN_PATH.end(); it_comeon_visualizer8++){
      geometry_msgs::Point p8;
      p8.x = (*it_comeon_visualizer8)->point->x();
      p8.y = (*it_comeon_visualizer8)->point->y();
      p8.z = (*it_comeon_visualizer8)->point->z();
      CHOSEN_PATH_points.points.push_back(p8);
      if((*it_comeon_visualizer8)->myParent != nullptr){
        if(*it_comeon_visualizer8 != *CHOSEN_PATH.begin()){
          CHOSEN_PATH_line_list.points.push_back(p8);
          p8.x = (*it_comeon_visualizer8)->myParent->point->x();
          p8.y = (*it_comeon_visualizer8)->myParent->point->y();
          p8.z = (*it_comeon_visualizer8)->myParent->point->z();
          CHOSEN_PATH_line_list.points.push_back(p8);
        }
      }
    }
    pub.chosen_path_visualization_pub.publish(CHOSEN_PATH_points);
    pub.chosen_path_visualization_pub.publish(CHOSEN_PATH_line_list);

  if(!ALC_POINTS.empty()){
    ALC_points.header.frame_id = ALC_line_list.header.frame_id = "world";
    ALC_points.ns = "points";
    ALC_points.action = visualization_msgs::Marker::ADD;
    ALC_points.pose.orientation.w = 1.0;
    ALC_points.id = 0;
    ALC_line_list.id = 1;
    ALC_points.type = visualization_msgs::Marker::POINTS;
    ALC_line_list.type = visualization_msgs::Marker::LINE_LIST;
    ALC_points.scale.x = 0.2;
    ALC_points.scale.y = 0.2;
    ALC_line_list.scale.x = 0.1;
    ALC_points.color.r = 0.8f;
    ALC_points.color.b = 0.8f;
    ALC_points.color.a = 1.0;
    ALC_line_list.color.b = 1.0;
    ALC_line_list.color.a = 1.0;
    std::list<node*>::iterator it_comeon_visualizer9;
    for(it_comeon_visualizer9 = ALC_POINTS.begin(); it_comeon_visualizer9 != ALC_POINTS.end(); it_comeon_visualizer9++){
      geometry_msgs::Point point1;
      point1.x = (*it_comeon_visualizer9)->point->x();
      point1.y = (*it_comeon_visualizer9)->point->y();
      point1.z = (*it_comeon_visualizer9)->point->z();
      ALC_points.points.push_back(point1);
      // geometry_msgs::Point point2;
      // point2.x = (*(std::next(it_comeon_visualizer9, 1)))->point->x();
      // point2.y = (*(std::next(it_comeon_visualizer9, 1)))->point->y();
      // point2.z = (*(std::next(it_comeon_visualizer9, 1)))->point->z();
          ALC_line_list.points.push_back(point1);
          // ALC_line_list.points.push_back(point2);
    }
    pub.alc_points_visualization_pub.publish(ALC_points);
    pub.alc_path_visualization_pub.publish(ALC_line_list);
  }

  if(!traversable.list.empty()){
    TRAVERSABLE_points.header.frame_id = "world";
    TRAVERSABLE_points.ns = "points";
    TRAVERSABLE_points.action = visualization_msgs::Marker::ADD;
    TRAVERSABLE_points.pose.orientation.w = 1.0;
    TRAVERSABLE_points.id = 0;
    TRAVERSABLE_points.type = visualization_msgs::Marker::SPHERE_LIST;
    TRAVERSABLE_points.scale.x = 0.6;
    TRAVERSABLE_points.color.r = 0.0f;
    TRAVERSABLE_points.color.g = 1.0f;
    TRAVERSABLE_points.color.b = 1.0f;
    TRAVERSABLE_points.color.a = 1.0;
    std::list<ufo::math::Vector3>::iterator it_comeon_visualizer2;	
    for(it_comeon_visualizer2 = traversable.list.begin(); it_comeon_visualizer2 != traversable.list.end(); it_comeon_visualizer2++){
      geometry_msgs::Point p;
      p.x = it_comeon_visualizer2->x();
      p.y = it_comeon_visualizer2->y();
      p.z = it_comeon_visualizer2->z();
      TRAVERSABLE_points.points.push_back(p);
    }
    pub.hits_pub.publish(TRAVERSABLE_points);
  }

  if(!local.list.empty()){
    local_frontier_nodes.header.frame_id = "world";
    local_frontier_nodes.ns = "points";
    local_frontier_nodes.action = visualization_msgs::Marker::ADD;
    local_frontier_nodes.pose.orientation.w = 1.0;
    local_frontier_nodes.id = 0;
    local_frontier_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    local_frontier_nodes.scale.x = 0.4;
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
    pub.local_nodes_pub.publish(local_frontier_nodes);
  }

  if(!global.list.empty()){
    global_frontier_nodes.header.frame_id = "world";
    global_frontier_nodes.ns = "points";
    global_frontier_nodes.action = visualization_msgs::Marker::ADD;
    global_frontier_nodes.pose.orientation.w = 1.0;
    global_frontier_nodes.id = 0;
    global_frontier_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    global_frontier_nodes.scale.x = 0.4;
    global_frontier_nodes.color.b = 0.8f;
    global_frontier_nodes.color.r = 0.8f;
    global_frontier_nodes.color.g = 0.8f;
    global_frontier_nodes.color.a = 0.6;
    std::list<ufo::math::Vector3>::iterator it_comeon_visualizer5;	
    for(it_comeon_visualizer5 = global.list.begin(); it_comeon_visualizer5 != global.list.end(); it_comeon_visualizer5++){
      geometry_msgs::Point p0;
      p0.x = it_comeon_visualizer5->x();
      p0.y = it_comeon_visualizer5->y();
      p0.z = it_comeon_visualizer5->z();
      global_frontier_nodes.points.push_back(p0);
    }
    pub.global_nodes_pub.publish(global_frontier_nodes);
  }

  if (!old_poses.list.empty()) {
    nav_msgs::Path exploration_path;
    for (auto visited_it = old_poses.list.begin(); visited_it != old_poses.list.end(); visited_it++) {
      geometry_msgs::PoseStamped past_pose;
      past_pose.header.stamp = ros::Time::now();
      past_pose.header.frame_id = "world";
      past_pose.pose.position.x = visited_it->x();
      past_pose.pose.position.y = visited_it->y();
      past_pose.pose.position.z = visited_it->z();
      past_pose.pose.orientation.x = 0;
      past_pose.pose.orientation.y = 0;
      past_pose.pose.orientation.z = 0;
      past_pose.pose.orientation.w = 0;
      exploration_path.poses.push_back(past_pose);
    }
    exploration_path.header.stamp = ros::Time::now();
    exploration_path.header.frame_id = "world";
    pub.explored_path.publish(exploration_path);
  }

}

//----------------------------------------- MAIN ---------------------------------------------------//


int main(int argc, char *argv[])
{ 
  ros::init(argc, argv, "ACL_EXPLORATION");
  ros::NodeHandle nh;
  pub.points_pub = nh.advertise<visualization_msgs::Marker>("RRT_NODES", 1);
  pub.chosen_path_visualization_pub = nh.advertise<visualization_msgs::Marker>("CHOSEN_RRT_PATH_VISUALIZATION", 1);
  pub.alc_points_visualization_pub = nh.advertise<visualization_msgs::Marker>("ALC_POINTS", 1);
  pub.alc_path_visualization_pub = nh.advertise<visualization_msgs::Marker>("ALC_PATH", 1);
  pub.position_pub = nh.advertise<visualization_msgs::Marker>("POSITION", 1);
  pub.best_frontier_pub = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird/reference", 1);
  pub.path_to_goal_pub = nh.advertise<nav_msgs::Path>("/hummingbird/path", 1);
  pub.explored_path = nh.advertise<nav_msgs::Path>("/exploration_path", 1);
  pub.hits_pub = nh.advertise<visualization_msgs::Marker>("Traversable_frontiers", 1);
  pub.local_nodes_pub = nh.advertise<visualization_msgs::Marker>("local_frontier_nodes_vis", 1);
  pub.global_nodes_pub = nh.advertise<visualization_msgs::Marker>("global_frontier_nodes_vis", 1);
  // pub.map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("acl_full_map", 11);
  ros::Subscriber map_sub = nh.subscribe("ufomap_mapping_server_node/map", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("/hummingbird/ground_truth/odometry", 1, odomCallback);
  ros::Subscriber front_sub = nh.subscribe("/frontiers_nodes", 1, frontiersCallback);
  ros::Subscriber homing_sub = nh.subscribe("/return_home", 1, homingCallback);
  ros::Subscriber explore_sub = nh.subscribe("/explore", 1, exploreCallback);
  ros::Rate rate(10);

  dynamic_reconfigure::Server<acl_exploration::ServerConfig> server;
  dynamic_reconfigure::Server<acl_exploration::ServerConfig>::CallbackType f;
  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);

  position_received = true;
  published.reference.header.stamp = ros::Time::now();
  published.reference.header.frame_id = "world";
  current.path.header.stamp = ros::Time::now();
  current.path.header.frame_id = "world";
  home.navigate = false;
  explore.navigate = false;


  while(ros::ok()){
    
    high_resolution_clock::time_point start_total = high_resolution_clock::now();
    
    /*
      main computations to be executed here on

    */
    
    visualize();

    if (firstTime || next_pub()) {

      firstTime = false;

      if (local.navigate and explore.navigate and !home.navigate) {

        exploreLocally();

      } else if (!local.navigate and global.navigate and explore.navigate and !home.navigate) {

        globalReposition();

      }

      if (home.navigate and !explore.navigate) {

        return_to_base_planning();

      }
    
    }

    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    
    // cout << "\033[;34m Total Execution time\033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// TODO LIST

/*

1) Fix frontiers detection -- done
2) Compute information gain -- TBD
3) Fix heading deviation based search -- done
4) Global graph based strategy -- done
5) Active loop closing -- candidates can be computed from the graph
6) Loop closing candidates to be included as part of hueristic function
7) Compute global path cost based on hueristic and shortest path 

*/
