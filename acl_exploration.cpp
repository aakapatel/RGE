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

using namespace std::chrono;
using namespace std;
using namespace std::this_thread;

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Structs

// struct rrtSolverStatus rrt_solve(struct rrtCache *instance,
//                                  double *u,
//                                  const double *params,
//                                  const double *y0,
//                                  const double *c0);
                                 
// struct rrtCache *rrt_new(void);



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

      

      
      int findInformationGain(float SCALER_AABB, float givenHorizontal, float givenVertical, float givenMin, float givenMax, ufo::map::OccupancyMapColor const& map, bool excludePath, bool findAnyInfo){
        if(myHits.empty() or findAnyInfo){
          // Setting up targets, X / Y targets in either positive (P) or negative (N) direction
          ufo::math::Vector3 targetXP(point->x() + 1, point->y(), point->z());
          ufo::math::Vector3 targetXN(point->x() - 1, point->y(), point->z());
          ufo::math::Vector3 targetYP(point->x(), point->y() + 1, point->z());
          ufo::math::Vector3 targetYN(point->x(), point->y() - 1, point->z());
          ufo::math::Vector3 upwards(point->x(), point->y(), point->z() + 1);
        
          //Angles
          double vertical_angle = givenVertical;
          double horizontal_angle = givenHorizontal;
        
          //Distances
          double near_distance = givenMin;
          double far_distance = givenMax;
        
          //Frustums
          ufo::geometry::Frustum frustXP(*point, targetXP, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          ufo::geometry::Frustum frustXN(*point, targetXN, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          ufo::geometry::Frustum frustYP(*point, targetYP, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          ufo::geometry::Frustum frustYN(*point, targetYN, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
          int checks = 0;
          for (auto it = map.beginLeaves(frustXP, false, false, true, false, 0), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 0)){
              if(findAnyInfo){
                return 1;
              }
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustXN, false, false, true, false, 0), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 0)){
              if(findAnyInfo){
                return 1;
              }
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustYP, false, false, true, false, 0), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 0)){
              if(findAnyInfo){
                return 1;
              }
              myHits.push_back(end_point);
            }
          }
          for (auto it = map.beginLeaves(frustYN, false, false, true, false, 0), it_end = map.endLeaves(); it != it_end; ++it){
            ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
            ufo::geometry::LineSegment myLine(*point, end_point);
            checks++;
            if(!isInCollision(map, myLine, true, false, false, 0)){
              if(findAnyInfo){
                return 1;
              }
              myHits.push_back(end_point);
            }
          }
          if(myParent != nullptr and not excludePath){
            myParent->findInformationGain(SCALER_AABB, givenVertical, givenHorizontal, givenMin, givenMax, map, excludePath, findAnyInfo);
          }
        }else{
          std::list<ufo::math::Vector3>::iterator it_hits;	
          for(it_hits = myHits.begin(); it_hits != myHits.end();){
            ufo::geometry::LineSegment myLine2(*point, *it_hits);
            if(isInCollision(map, myLine2, true, false, false, 3)){
              it_hits = myHits.erase(it_hits);
            }else{
              if(isInCollision(map, *it_hits, false, true, false, 3)){
                it_hits = myHits.erase(it_hits);
              }else{
                it_hits++;
              };
            };
          };
        }
        std::list<ufo::math::Vector3> myTotalHits{};
        // addHits(&myTotalHits);
        // int hits = myTotalHits.size();
        // return hits;
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

// End of structs
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Variables

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
bool position_received = false;
bool fetched_path = false;
bool newPath = false;
bool allowNewPath = true;
bool recoveryUnderway = false;
double SENSOR_RANGE;
double SENSOR_MIN;
double SENSOR_VERTICAL;
double SENSOR_HORIZONTAL;
double SCALER_INFORMATION_GAIN;
double SCALER_DISTANCE;
double SCALER_ACTUATION;
double NEXT_POINT_DISTANCE;
double NEXT_PATH_DISTANCE;
double NMPC_DT;
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

// Ufomap
ufo::map::OccupancyMapColor myMap(RESOLUTION);

// Lists
std::list<struct node*> RRT_TREE{};
std::list<struct node*> CHOSEN_PATH{};
std::list<double> CHOSEN_PATH_VREF{};
std::list<double>::iterator vref_itterator;
std::list<struct node*> ALL_PATH{};
std::list<struct node*> myGoals{};
std::list<struct node*> myReserveGoals{};
std::list<ufo::math::Vector3> hits{};
std::list<node*> VISITED_POINTS{};
std::list<node*>::iterator path_itterator;

// End of variables
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Functions




// Publish all visualization messages here.

void visualize(ros::Publisher* position_pub, ros::Publisher* points_pub, ros::Publisher* map_pub){
  visualization_msgs::Marker POSITION_point;

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
}


// Collision checking

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
}

// End of callback functions
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// Main


int main(int argc, char *argv[])
{ 
  // Subscribers and publishers
  ros::init(argc, argv, "ACL_EXPLORATION");
  ros::NodeHandle nh;
  ros::Publisher points_pub = nh.advertise<visualization_msgs::Marker>("RRT_NODES", 1);
  ros::Publisher position_pub = nh.advertise<visualization_msgs::Marker>("POSITION", 1);
  // ros::Publisher chosen_path_pub = nh.advertise<geometry_msgs::PoseStamped>("/shafter3d/reference", 1);
  // ros::Publisher chosen_path_pub = nh.advertise<nav_msgs::Odometry>("/pelican/reference", 1);

  ros::Publisher map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("acl_full_map", 11);
  ros::Subscriber map_sub = nh.subscribe("ufomap_mapping_server_node/map", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("/pelican/ground_truth/odometry", 1, odomCallback);
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


    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<microseconds>(stop_total - start_total);
    cout << "\nExecution time: " << duration_total.count() << " micro seconds for " << myGoals.size() << " path/s." << endl;
    visualize(&points_pub, &map_pub, &position_pub);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// TODO LIST

/*

1) Fix frontiers detection
2) Compute information gain
3) Fix heading deviation based search 
4) Active loop closing 

*/
