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


// The node struct.
// This struct represents the different kinds of points in the rrt, this includes:
// the nodes in the rrt tree, the goals and the paths.

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

struct Frontier_main {
  std::list<ufo::math::Vector3> list;
  ufo::math::Vector3 position;
  geometry_msgs::PoseStamped pose;
  double heading;
  nav_msgs::Path path;
};

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

std::vector<Point> Astar(Point & start_pos, Point & end_pos, std::vector<Node> all_nodes)
{
    // Find the start and end nodes
    Node start_node;
    Node end_node;

    for (Node & node : all_nodes)
    {
        if (node.myposition == end_pos) // start and end swaped here because A-Star traces from end to start
            start_node = node;
        if (node.myposition == start_pos)
            end_node = node;
    }

    std::vector<Point> found_path; // Output path


    std::set<Node> open_list; // Using set to be ordered
    std::vector<Point> closed_list;

    open_list.emplace(start_node);


    while (!open_list.empty())
    {
        // Use node with lowest total_dist (the set is always sorted)
        Node current_node = *open_list.begin();

        // Remove smallest and add to closed list
        open_list.erase(open_list.begin());
        closed_list.push_back(current_node.myposition);


        // Found the goal
        if (current_node.myposition == end_node.myposition)
        {
            // Found the goal
            Node * parent = &current_node;

            // Jump from parent to parent to trace back
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

            // Child is on the closed list
            if (std::find_if(closed_list.begin(), closed_list.end(), [&child](Point & pt) { return(pt == child.myposition); }) != closed_list.end())
                continue;

            // Create the total_dist, dist_to_start, and heurist_dist_to_end values for the new subnode

            // Calculated the total distance to the start by adding the distance from the subnode to the current node.
            // Note: This distance could be precomputed and just held together with the "connections"
            child.dist_to_start = current_node.dist_to_start + child.myposition.DistTo(current_node.myposition);


        // Total cost is the sum of past distance and the heuristic (which assumes just line-of-flight)
        child.total_dist = child.dist_to_start + child.myposition.DistTo(end_node.myposition);

            // Child is already in the open list
            if (std::find_if(open_list.begin(), open_list.end(), [&child](const Node & node) { return((node == child) && (child.dist_to_start > node.dist_to_start)); }) != open_list.end())
                continue;

            // Add the child to the open list
            open_list.emplace(child);
        }

    }

    return(found_path);
}





int diy;
// Variables
int NUMBER_OF_NODES;
int NUMBER_OF_GOALS;
int NUMBER_OF_ITTERATIONS;
int NMPC_POINTS;
int itterations;
int advance_index = 0;
float DISTANCE_BETWEEN_NODES; // 1.0;
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

bool frontiers_received;
Frontier_main all;
Frontier_main local;
Frontier_main global;
Frontier_main current;
Frontier_main old_poses;
Frontier_main traversable;
ufo::math::Vector3 selectedfrontier;
ufo::math::Vector3 selectedgfrontier;
ufo::math::Vector3 nearest_to_global;
double nearest_to_global_id;
geometry_msgs::PoseStamped best_frontier;
nav_msgs::Odometry m_currentPose;
nav_msgs::Odometry m_prevPose;


ufo::map::OccupancyMapColor myMap(RESOLUTION);
std::list<ufo::math::Vector3> ext;
std::list<ufo::math::Vector3> accessible;
std::list<ufo::math::Vector3> positionslist;
old_poses_list_t akash;
old_poses_list_t visited_and_global;
graph_vector_t COMPLETE_GRAPH;

std::list<struct node*> RRT_TREE{};
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

//--------------------------------- FUNCTIONS ----------------------------------------------//

void linSpace(node* givenNode, float givenDistance){
  ufo::math::Vector3 newVector(givenNode->myParent->point->x() - givenNode->point->x(), givenNode->myParent->point->y() - givenNode->point->y(), givenNode->myParent->point->z() - givenNode->point->z());
  float distance = newVector.norm();
  float itterations = (distance / givenDistance);
  float part = givenDistance / distance;
  float xStep = (givenNode->myParent->point->x() - givenNode->point->x()) * part;
  float yStep = (givenNode->myParent->point->y() - givenNode->point->y()) * part;
  float zStep = (givenNode->myParent->point->z() - givenNode->point->z()) * part;
  node* parent = givenNode->myParent;
  node* nextNode = givenNode->myParent;
  for(int i = 1; i < itterations; i++){
    node* newPoint = new node(givenNode->myParent->point->x() - i * xStep, givenNode->myParent->point->y() - i * yStep, givenNode->myParent->point->z() - i * zStep);
    newPoint->addParent(parent);
    parent = newPoint;
    RRT_TREE.push_back(newPoint);
  }
  givenNode->addParent(parent);
  if(nextNode->myParent != nullptr){
    linSpace(nextNode, givenDistance);
  }
}

// Evaluates the current point in the current path.
// This includes deciding when to change the current target to the next node in the path and when to calculate a new path.
void evaluateCurrentPoint(ros::Publisher* chosen_path_pub){
  if((sqrt(pow(position_x - goalNode->point->x(), 2) + pow(position_y - goalNode->point->y(), 2) + pow(position_z - goalNode->point->z(), 2)) < NEXT_PATH_DISTANCE)){ //Here
    itterations = 0;
    fetched_path = false;
    RRT_created = false;
    GOALS_generated = false;
    position_received = false;
    allowNewPath = true;
  }
  if((sqrt(pow(position_x - currentTarget->point->x(), 2) + pow(position_y - currentTarget->point->y(), 2) + pow(position_z - currentTarget->point->z(), 2)) < NEXT_POINT_DISTANCE) and path_itterator != --CHOSEN_PATH.end()){ //Here
    advance_index++;
    path_itterator = CHOSEN_PATH.begin();
    std::advance(path_itterator, advance_index);
    currentTarget = *path_itterator;
    if(path_itterator == CHOSEN_PATH.end()){
      path_itterator--;
      currentTarget = *path_itterator;
    }
  }
  if(path_itterator != CHOSEN_PATH.end()){
    nav_msgs::Odometry nextPoint;
    nextPoint.pose.pose.position.x = (currentTarget)->point->x();
    nextPoint.pose.pose.position.y = (currentTarget)->point->y();
    nextPoint.pose.pose.position.z = (currentTarget)->point->z();
    if(!recoveryUnderway){
      nextPoint.twist.twist.linear.x = *vref_itterator;
      vref_itterator++;
      nextPoint.twist.twist.linear.y = *vref_itterator;
      vref_itterator++;
      nextPoint.twist.twist.linear.z = *vref_itterator;
      vref_itterator++;
    }else{
      nextPoint.twist.twist.linear.x = 0;
      nextPoint.twist.twist.linear.y = 0;
      nextPoint.twist.twist.linear.z = 0;
    }
    nextPoint.pose.pose.orientation.x = 0;
    nextPoint.pose.pose.orientation.y = 0;
    nextPoint.pose.pose.orientation.z = 0;
    nextPoint.pose.pose.orientation.w = 0;
    nextPoint.header.stamp = ros::Time::now();
    nextPoint.header.frame_id = "world";
    chosen_path_pub->publish(nextPoint);
  }
}


// Tunes the generation.
// This sets the bounding box which represents the local space.
// The bounding box will be reduced in size as much as possible without losing free space.
void tuneGeneration(ufo::map::OccupancyMapColor const& map, bool occupied_space, bool free_space, bool unknown_space, float given_x, float given_y, float given_z, ufo::map::DepthType min_depth = 3){
  highest_x = -9999;
  highest_y = -9999;
  highest_z = -9999;
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();
  ufo::math::Vector3 minPoint(given_x - 1 * SCALER_AABB, given_y - 1 * SCALER_AABB, given_z - 1 * SCALER_AABB);
  ufo::math::Vector3 maxPoint(given_x + 1 * SCALER_AABB, given_y + 1 * SCALER_AABB, given_z + 1 * SCALER_AABB);
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

// Evaluates the RRT tree for the purpose of reducing the distance in a path.
// The node with the shortest path to the root node, which the goal in question can see, will be the parent of the currently evaluated goal node.
// The path between the goal node and its' parent has to pass a series of sphere checks, which aims to guarantee a distance of RADIOUS between the path and the occupied and unknown space.
void findShortestPath(node* selectednode){
  // for(std::list<node*>::iterator it_goals = myGoals.begin(); it_goals != myGoals.end(); it_goals++){
    struct node* chosenNode = nullptr;
    double distance = std::numeric_limits<double>::max();
    for(std::list<node*>::iterator it_RRT = RRT_TREE.begin(); it_RRT != RRT_TREE.end(); it_RRT++){
      double distanceNodeToGoal = sqrt(pow((*it_RRT)->point->x() - (selectednode)->point->x(), 2) + pow((*it_RRT)->point->y() - (selectednode)->point->y(), 2) + pow((*it_RRT)->point->z() - (selectednode)->point->z(), 2));
      double distanceToNode = (*it_RRT)->sumDistance();
      double totalDistance = distanceNodeToGoal + distanceToNode;
      if(totalDistance < distance){
        ufo::geometry::LineSegment myLine(*((selectednode)->point), *((*it_RRT)->point));
        if(!isInCollision(myMap, myLine, true, false, true, 3)){
          ufo::math::Vector3 newVector((selectednode)->point->x() - (*it_RRT)->point->x(), (selectednode)->point->y() - (*it_RRT)->point->y(), (selectednode)->point->z() - (*it_RRT)->point->z());
          float distance_calc = newVector.norm();
          float itterations = (distance_calc / DISTANCE_BETWEEN_NODES);
          float part = DISTANCE_BETWEEN_NODES / distance_calc;
          float xStep = ((selectednode)->point->x() - (*it_RRT)->point->x()) * part;
          float yStep = ((selectednode)->point->y() - (*it_RRT)->point->y()) * part;
          float zStep = ((selectednode)->point->z() - (*it_RRT)->point->z()) * part;
          bool add = true;
          for(int i = 1; i < itterations; i++){
            ufo::math::Vector3 newVector = ufo::math::Vector3((*it_RRT)->point->x() + i * xStep, (*it_RRT)->point->y() + i * yStep, (*it_RRT)->point->z() + i * zStep);
            ufo::geometry::Sphere new_sphere(newVector, RADIOUS);
            if(isInCollision(myMap, new_sphere, true, false, true, 3)){
              add = false;
              break;
            }
          }
          if(add){
            distance = totalDistance;
            chosenNode = *it_RRT;
            
          }
        }
      }

    }

    if(chosenNode != nullptr){
      (selectednode)->addParent(chosenNode);
      chosenNode->addChild(selectednode);
      CHOSEN_PATH.push_back(chosenNode);
    }
  // }
}

// Generates the RRT
// Generates the nodes in the RRT-tree and connects them to a already existing parent.
// The root of the node exists at the current position of the plant.
void generateRRT(float given_x, float given_y, float given_z){
  for(std::list<node*>::iterator it_clear_helper = RRT_TREE.begin(); it_clear_helper != --RRT_TREE.end(); it_clear_helper++){
    (*it_clear_helper)->readyForDeletion();
    delete(*it_clear_helper);
  }
  for(std::list<node*>::iterator it_clear_helper = myGoals.begin(); it_clear_helper != --myGoals.end(); it_clear_helper++){
    (*it_clear_helper)->addParent(nullptr);
  }
  RRT_TREE.clear();
  std::cout << "Building graph...\n " << std::endl;
  node* origin = new node(given_x, given_y, given_z);
  origin->addParent(nullptr);
  RRT_TREE.push_back(origin);
  srand(time(0));
  itterations = 0;
  while(((RRT_TREE.size() <= 10000 and RUN_BY_NODES) or (itterations <= NUMBER_OF_ITTERATIONS and !RUN_BY_NODES)) and itterations < 30000){
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
      for(it_node = RRT_TREE.begin(); it_node != RRT_TREE.end(); it_node++){
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
          RRT_TREE.push_back(new_node);
      }
    };
    itterations++;
  };
  std::cout << "Graph built successfully with " << RRT_TREE.size() << " number of nodes\n" << std::endl;
  
  if(RUN_BY_NODES){
    // std::cout << "Verifying tree" << std::endl;
    int total_childs = 0;
    int total_parents = 0;
    std::list<node*>::iterator it_comeon;
    for(it_comeon = RRT_TREE.begin(); it_comeon != RRT_TREE.end(); it_comeon++){
      total_childs = total_childs + (*it_comeon)->myChilds.size();
      if((*it_comeon)->myParent != nullptr){
        total_parents++;
      }
    };
    if(total_childs != 0){
      RRT_created = true;
    }
  }
};




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
      ufo::math::Vector3 direction = goal - position;
      ufo::math::Vector3 center = position + (direction / 2.0);
      double distance = direction.norm();
      direction /= distance;
      double yaw = -atan2(direction[1], direction[0]);
      double pitch = -asin(direction[2]);
      double roll = 0;  // TODO: Fix
      ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.5, 0.5),
              ufo::math::Quaternion(roll, pitch, yaw));
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
    if (((angle < 40) && (angle > -40)) && distance < 15 && distance > 4) {
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
  for (auto rit = RRT_TREE.begin(); rit != RRT_TREE.end(); rit++) {

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

  for (auto tree_it = RRT_TREE.begin(); tree_it != RRT_TREE.end(); tree_it++) {

    for (auto ray_it = successful_ray.begin(); ray_it != successful_ray.end(); ray_it++) {

      double d = calculateDistance(*ray_it, *((*tree_it)->point));

      if (d < 0.2) {
        alc_candidates.push_back(*tree_it);
      }

    }

  }

return alc_candidates;  

}


// void setGlobalPath() {

//   for (auto tree_iterator = RRT_TREE.begin(); tree_iterator != RRT_TREE.end(); tree_iterator++) {





//   }



// }

bool next_pub(const geometry_msgs::PoseStamped &best_frontier) {

  ufo::math::Vector3 pos;
  pos.x() = best_frontier.pose.position.x;
  pos.y() = best_frontier.pose.position.y;
  pos.z() = best_frontier.pose.position.z;
  double pub_dis = calculateDistance(pos, current.position);
  if (pub_dis < 3) {
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
  for (it = visited.begin(); it != visited.end(); it++) {
    ufo::math::Vector3 direction = it->first - global_fronier;
    ufo::math::Vector3 center = global_fronier + (direction / 2.0);
    double distance = direction.norm();
    direction /= distance;
    double yaw = -atan2(direction[1], direction[0]);
    double pitch = -asin(direction[2]);
    double roll = 0;  // TODO: Fix
    ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 1, 1),
            ufo::math::Quaternion(roll, pitch, yaw));
    double m = calculateDistance(it->first, global_fronier);
    num = m;
    if (num < mynum && !isInCollision(map, obb, true, false, false, 0)) {
      mynum = num;
      cp = std::make_pair(it->first, it->second);
    }
  }
  return cp;
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
      ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.8, 0.8),
              ufo::math::Quaternion(roll, pitch, yaw));
      if (!isInCollision(map, obb, true, false, false, 0) && distance < 10) {
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
  for (geometry_msgs::Point frontier : msg->points) {
    frontier_point = ufo::math::Vector3(frontier.x, frontier.y, frontier.z);
    double radius = 1;
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

}

void visualize(ros::Publisher* points_pub, ros::Publisher* chosen_path_visualization_pub, ros::Publisher* alc_points_visualization_pub,
                                                  ros::Publisher* alc_path_visualization_pub, ros::Publisher* position_pub, ros::Publisher* map_pub, 
                                                      ros::Publisher* hits_pub, ros::Publisher* local_nodes_pub, ros::Publisher* global_nodes_pub,
                                                              ros::Publisher global_connection_visualization_pub,
                                                                    ros::Publisher global_connection_line_pub){
  visualization_msgs::Marker RRT_points, RRT_line_list, CHOSEN_PATH_points, CHOSEN_PATH_line_list, ALC_points, ALC_line_list, POSITION_point, HITS_points, local_frontier_nodes, global_frontier_nodes, global_connection_points, global_connection_line_list;

  if(1==1){
    RRT_points.header.frame_id = RRT_line_list.header.frame_id = "world";
    RRT_points.ns = "points";
    RRT_points.action = visualization_msgs::Marker::ADD;
    RRT_points.pose.orientation.w = 1.0;
    RRT_points.id = 0;
    RRT_line_list.id = 1;
    RRT_points.type = visualization_msgs::Marker::POINTS;
    RRT_line_list.type = visualization_msgs::Marker::LINE_LIST;
    RRT_points.scale.x = 0.2;
    RRT_points.scale.y = 0.2;
    RRT_line_list.scale.x = 0.1;
    RRT_points.color.g = 1.0f;
    RRT_points.color.a = 1.0;
    RRT_line_list.color.b = 1.0;
    RRT_line_list.color.a = 1.0;
    std::list<node*>::iterator it_comeon_visualizer;
    for(it_comeon_visualizer = RRT_TREE.begin(); it_comeon_visualizer != RRT_TREE.end(); it_comeon_visualizer++){
      geometry_msgs::Point p;
      p.x = (*it_comeon_visualizer)->point->x();
      p.y = (*it_comeon_visualizer)->point->y();
      p.z = (*it_comeon_visualizer)->point->z();
      RRT_points.points.push_back(p);
      if((*it_comeon_visualizer)->myParent != nullptr){
        RRT_line_list.points.push_back(p);
        p.x = (*it_comeon_visualizer)->myParent->point->x();
        p.y = (*it_comeon_visualizer)->myParent->point->y();
        p.z = (*it_comeon_visualizer)->myParent->point->z();
        RRT_line_list.points.push_back(p);
      }
    }
    points_pub->publish(RRT_points);
    points_pub->publish(RRT_line_list);
  }

  if(1==1){
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
    chosen_path_visualization_pub->publish(CHOSEN_PATH_points);
    chosen_path_visualization_pub->publish(CHOSEN_PATH_line_list);
  }


  if(ALC_POINTS.size() > 0){
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
    alc_points_visualization_pub->publish(ALC_points);
    alc_path_visualization_pub->publish(ALC_line_list);
  }



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

//----------------------------------------- MAIN ---------------------------------------------------//


int main(int argc, char *argv[])
{ 
  ros::init(argc, argv, "ACL_EXPLORATION");
  ros::NodeHandle nh;
  ros::Publisher points_pub = nh.advertise<visualization_msgs::Marker>("RRT_NODES", 1);
  ros::Publisher chosen_path_visualization_pub = nh.advertise<visualization_msgs::Marker>("CHOSEN_RRT_PATH_VISUALIZATION", 1);
  ros::Publisher alc_points_visualization_pub = nh.advertise<visualization_msgs::Marker>("ALC_POINTS", 1);
  ros::Publisher alc_path_visualization_pub = nh.advertise<visualization_msgs::Marker>("ALC_PATH", 1);
  ros::Publisher position_pub = nh.advertise<visualization_msgs::Marker>("POSITION", 1);
  ros::Publisher best_frontier_pub = nh.advertise<geometry_msgs::PoseStamped>("/hummingbird/reference", 1);
  ros::Publisher path_to_goal_pub = nh.advertise<nav_msgs::Path>("/hummingbird/path", 1);
  ros::Publisher global_connection_visualization_pub = nh.advertise<visualization_msgs::Marker>("/points_global_connection", 1);
  ros::Publisher global_connection_line_pub = nh.advertise<visualization_msgs::Marker>("/line_global_connection", 1);
  ros::Publisher hits_pub = nh.advertise<visualization_msgs::Marker>("Traversable_frontiers", 1);
  ros::Publisher local_nodes_pub = nh.advertise<visualization_msgs::Marker>("local_frontier_nodes_vis", 1);
  ros::Publisher global_nodes_pub = nh.advertise<visualization_msgs::Marker>("global_frontier_nodes_vis", 1);
  ros::Publisher map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("acl_full_map", 11);
  ros::Subscriber map_sub = nh.subscribe("ufomap_mapping_server_node/map", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("/hummingbird/ground_truth/odometry", 1, odomCallback);
  ros::Subscriber front_sub = nh.subscribe("/frontiers_nodes", 1, frontiersCallback);
  ros::Rate rate(10);
  // Dynamic reconfigure
  dynamic_reconfigure::Server<acl_exploration::ServerConfig> server;
  dynamic_reconfigure::Server<acl_exploration::ServerConfig>::CallbackType f;
  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);

  fetched_path = true;
  RRT_created = true;
  GOALS_generated = true;
  position_received = true;


  while(ros::ok()){
    
    high_resolution_clock::time_point start_total = high_resolution_clock::now();
    traversable.list = extractUnknowns(myMap, local.list, current.position, true, true, true, 0);
    /*
      main computations to be executed here on

    */
      // tuneGeneration(myMap, false, true, false, current.position.x(), current.position.y(), current.position.z(), 0);
      // generateRRT(current.position.x(), current.position.y(), current.position.z());

      best_frontier.header.stamp = ros::Time::now();
      best_frontier.header.frame_id = "world";
      current.path.header.stamp = ros::Time::now();
      current.path.header.frame_id = "world";


    if (firstTime || next_pub(best_frontier)) {
        
        if (traversable.list.size() > 5) {
            firstTime = false;
            selectedfrontier = selectLocalFrontier(traversable.list, current.position, current.heading);
            best_frontier = publish_goal(selectedfrontier);
            best_frontier_pub.publish(best_frontier);
            ROS_INFO("Local Frontier Published");
            current.path.poses.push_back(current.pose);
            current.path.poses.push_back(best_frontier);
            path_to_goal_pub.publish(current.path);
            current.path.poses.clear();
        }
      if (traversable.list.size() < 3 and global.list.size() > 0) {
                tuneGeneration(myMap, false, true, false, current.position.x(), current.position.y(), current.position.z(), 0);
                generateRRT(current.position.x(), current.position.y(), current.position.z());
                node* rrt_pointer;
                rrt_pointer = selectGlobalFrontier(global.list);
                cout << "\nGoal Node X : " << ((rrt_pointer)->point)->x() << endl;
                cout << "\nGoal Node Y : " << ((rrt_pointer)->point)->y() << endl;
                cout << "\nGoal Node Z : " << ((rrt_pointer)->point)->z() << endl;
                ALC_POINTS = selectGlobalPath(global.list);

                // cout << "\nloop closure points list size is: " << ALC_POINTS.size()<< endl;
                // cout << "\nPath Consist of : " << CHOSEN_PATH.size() << " Points "<< endl;
              
              high_resolution_clock::time_point start_total = high_resolution_clock::now();
              
              if (RRT_TREE.size() > 0) {

                std::vector<Node> all_nodes;
                std::vector<Node> my_nodes;
                
                all_nodes.clear();
                my_nodes.clear();

                Node sample_nd;
                Node sample_nd2;

                for (auto insert_helper = RRT_TREE.begin(); insert_helper != RRT_TREE.end(); insert_helper++) {

                  my_nodes.push_back( { Point{ (((*insert_helper)->point)->x()), (((*insert_helper)->point)->y()), (((*insert_helper)->point)->z()) } } );

                }

                cout << "\nMY nodes size is: " << my_nodes.size() << endl;

                for (auto graph_it = my_nodes.begin(); graph_it != --my_nodes.end(); graph_it++) {

                  for (auto graph_it2 = std::next(my_nodes.begin(), 1); graph_it2 != my_nodes.end(); graph_it2++) {
                    
                    ufo::math::Vector3 d1((*graph_it).myposition.x, (*graph_it).myposition.y, (*graph_it).myposition.z);
                    ufo::math::Vector3 d2((*graph_it2).myposition.x, (*graph_it2).myposition.y, (*graph_it2).myposition.z);

                    ufo::math::Vector3 direction = d2 - d1;
                    ufo::math::Vector3 center = d1 + (direction / 2.0);
                    double distance = direction.norm();
                    direction /= distance;
                    double yaw = -atan2(direction[1], direction[0]);
                    double pitch = -asin(direction[2]);
                    double roll = 0;  // TODO: Fix
                    ufo::geometry::OBB obb(center, ufo::math::Vector3(distance / 2.0, 0.5, 0.5),
                            ufo::math::Quaternion(roll, pitch, yaw));
                    double rang = calculateDistance(d1, d2);
                    if (!isInCollision(myMap, obb, true, false, true, 0) && rang > 1 && rang < 4.5) {

                      sample_nd = *graph_it;
                      sample_nd2 = *graph_it2;

                      sample_nd.AddConnection(sample_nd2);
                      sample_nd2.AddConnection(sample_nd);
                      
                      all_nodes.push_back(sample_nd);
                      all_nodes.push_back(sample_nd2);

                    }

                  }

                }


                
                double close_dis;
                double end_dis;
                double my_end_dis = 100000000;
                double my_close_dis = 100000000;
                double a1, b1, c1;
                double a2, b2, c2;
                for (auto closest_it = all_nodes.begin(); closest_it != all_nodes.end(); closest_it++) {
                  
                  close_dis = std::sqrt(std::pow((*closest_it).myposition.x - current.position.x(), 2) + 
                                        std::pow((*closest_it).myposition.y - current.position.y(), 2) +
                                        std::pow((*closest_it).myposition.z - current.position.z(), 2));

                  end_dis = std::sqrt(std::pow((*closest_it).myposition.x - ((rrt_pointer)->point)->x(), 2) + 
                                        std::pow((*closest_it).myposition.y - ((rrt_pointer)->point)->y(), 2) +
                                        std::pow((*closest_it).myposition.z - ((rrt_pointer)->point)->z(), 2));
                                        
                  if (close_dis < my_close_dis) {
                    my_close_dis = close_dis;
                    a1 = ((*closest_it).myposition.x);
                    b1 = ((*closest_it).myposition.y);
                    c1 = ((*closest_it).myposition.z);
                  }

                  if (end_dis < my_end_dis) {
                    my_end_dis = end_dis;
                    a2 = ((*closest_it).myposition.x);
                    b2 = ((*closest_it).myposition.y);
                    c2= ((*closest_it).myposition.z);
                  }

                }

                Point start_pos { a1, b1, c1 };
                Point end_pos { a2, b2, c2 };

                cout << "\nAll nodes size is: " << all_nodes.size() << endl;

                auto my_path = Astar(start_pos, end_pos, all_nodes);

                if (my_path.empty())
                    std::cout << "\nNo good path :( \n";
                else
                {
                    std::cout << "\nPath found :) \n";
                    // std::for_each(my_path.begin(), my_path.end(), [](const Point & pt) { pt.Print("Move to: "); });
                }

                high_resolution_clock::time_point stop_total = high_resolution_clock::now();
                auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
                cout << "\033[;34mExecution time\033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;
                
                
                cout<< "\033[;32mShortest path iterates through vertices:\033[0m\n ";
                for(auto vert_it = std::next(my_path.begin(),1); vert_it != my_path.end(); vert_it++){
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
                  // current.path.poses.push_back(current.pose);
                  current.path.poses.push_back(global_nodes);
                }
                path_to_goal_pub.publish(current.path);
                current.path.poses.clear();
                ufo::math::Vector3 lastpub(a2, b2, c2);
                best_frontier = publish_goal(lastpub); 

              }

      }

  }
    // high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    // auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    
    // cout << "\033[;34mExecution time\033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;
    
    visualize(&points_pub, &chosen_path_visualization_pub, &alc_points_visualization_pub, &alc_path_visualization_pub, &map_pub, &position_pub, &hits_pub, &local_nodes_pub, &global_nodes_pub, global_connection_visualization_pub,
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
4) Global graph based strategy -- in process
5) Active loop closing --TBD

*/
