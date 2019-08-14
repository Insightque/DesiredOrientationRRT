#pragma once

#ifndef Q_MOC_RUN
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <GeometricUtils.h>
#include <MathParam.h>
#include <AngleUtils.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <iostream>
#include <mutex>
#endif

namespace planner {

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef ob::SE2StateSpace::StateType STATETYPE;
typedef ob::SE2StateSpace STATESPACE;

class DecisionMaker {
 private:
  typedef pcl::PointXYZI VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  og::SimpleSetup* ss_g;
  ob::StateSpacePtr g_space;
  ob::RealVectorBounds* BOUNDS;
  ob::PlannerStatus g_solved;
  vector<vector<VectorXd> > g_map;

  // for custom C blocking
  double block_pos_x;
  double block_pos_y;
  tf::Quaternion block_quat;
  bool b_block;

  bool b_DORRT_STAR;

  vector<Vector3d> vPath_g;
  vector<Vector2d> OBSTACLE;

  double PLANNINGTIME;
  double COSTTHRESHOLD;
  double RESOLUTION;
  double START_G[3];
  double GOAL_G[3];
  double RANGE_OBS;
  double K_REP_OBS;
  double RANGE_REP;
  double K_REP;
  double K_ATT;

  bool b_goal;

  double CAR_C2R;

  vector<Vector2d> vObstacle;

  // Publishers & Subscribers
  ros::Publisher pub_c_blocking;
  ros::Publisher pub_path_rrt;

  ros::Subscriber sub_potential_array;
  ros::Subscriber sub_target_parking_space;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  std::mutex mutex_dm;
 public:
  DecisionMaker(ros::NodeHandle nh, ros::NodeHandle priv_nh);

  ~DecisionMaker(){}

  ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

  std::vector<pcl::PointXYZ> SearchNodeByRadius(pcl::PointXYZ searchPoint, float radius);

  int SearchNearestNodeIdxByRadius(pcl::PointXYZ searchPoint, float radius);

  void magneticVectorForce(const double* init, const double* target, double *B);

  double computeRepulsiveForce(double K, double dist, double range, double x, double tar_x);

  double computeAttractiveForce(double K, double x, double tar_x);

  VectorXd ComputePotentialField2(double x, double y, double yaw, double* start, double* goal);

  VectorXd ComputeObstacleField(double x, double y);

  bool DO( double* from_d, double* to_d, double* target_d);

  bool DesiredOrientation(ob::State* from, ob::State* to, ob::State* target);

  VectorXd magneticfield(ob::State* target, ob::State* nouse1, ob::State* nouse2);

  bool randomCheck(ob::State* rand);

  void VisualizePath();

  void UpdateGlobalPathData();

  void plan_init(og::SimpleSetup* ss,double* start, double* goal);

  void plan(bool b_goal);

  void points_obstacle_registered_callback(const VPointCloud::ConstPtr& msg,
                                           tf::TransformListener *listener,
                                           tf::StampedTransform *transform_);

  void target_parking_space_callback(const geometry_msgs::PoseStamped::ConstPtr& msg,
                                     tf::TransformListener *listener,
                                     tf::StampedTransform *transform_);
 public:
  // KD TREE : Obstacle
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree;
  pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree;

  // KD TREE : Path
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree_Path;
  pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree_Path;
  double SAFEREGION;
};


}  // end of namespace planner
