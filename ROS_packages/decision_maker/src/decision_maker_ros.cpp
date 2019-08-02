#ifndef Q_MOC_RUN
#include "../inc/quadtree.h"
#include "../inc/ebandmanager.h"

#include <ros/ros.h>

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

#include <boost/timer/timer.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>

#include <fstream>
#include <iostream>
#include <cmath>

#define DRAW
//#define DRAWVEHICLE
//#define CONFIG_ANALYSIS

#endif

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef ob::SE2StateSpace::StateType STATETYPE;
typedef ob::SE2StateSpace STATESPACE;

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

og::SimpleSetup* ss_g;
og::SimpleSetup* ss_l;

ob::StateSpacePtr g_space(new ob::DubinsStateSpace(5.88, true)); // false: forward
ob::StateSpacePtr g_space_local(new ob::DubinsStateSpace(5.88, true)); // false: forward

// for custom C blocking
double block_pos_x;
double block_pos_y;
tf::Quaternion block_quat;
bool b_block;

int b_DORRT_STAR = false;
vector<Vector3d> vPath_g;

double PLANNINGTIME = 10.0;
double COSTTHRESHOLD = 0.0;
double RESOLUTION = 0.3;

vector<Vector2d> OBSTACLE;

double START_G[3]={0.0,0.0,RADIANS(0)};
double GOAL_G[3]={0.0,0.0,RADIANS(0)};

//For local planner
double START_L[3]={0.0,0.0,RADIANS(0)};
double GOAL_L[3]={0.0,0.0,RADIANS(0)};

ob::RealVectorBounds BOUNDS(2);

double SAFEREGION = 0.6;

double RANGE_OBS = 30.0;
double K_REP_OBS = 10.0f;
double RANGE_REP= 10.0;
double K_REP = 10.0;
double K_ATT = 0.02;

vector<vector<VectorXd> > g_map; //unuse

ob::PlannerStatus g_solved;

bool b_goal;

double VEHICLEPOS_TMP[4]={0,0,0,0};

// KD TREE : Obstacle
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree;
pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree;

// KD TREE : Path
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree_Path;
pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree_Path;

// Publishers & Subscribers
ros::Publisher g_msgpub1;
ros::Publisher g_msgpub2;
ros::Publisher g_msgpub3;
ros::Publisher pub_path_rrt;

ros::Subscriber sub_potential_array;
ros::Subscriber sub_target_parking_space;

geometry_msgs::PoseArray g_posArray1;
geometry_msgs::PoseArray g_posArray2;
geometry_msgs::PoseArray g_posArray3;

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
  auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
  obj->setCostThreshold(ob::Cost(COSTTHRESHOLD));
  return obj;
}

int SearchNearestNodeIdxByRadius(pcl::PointXYZ searchPoint, float radius)
{
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  std::vector<pcl::PointXYZ> pvNode;

  if( g_kdTree_Path.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
  {
    return pointIdxRadiusSearch[0];
  }
  return -1;
}

std::vector<pcl::PointXYZ> SearchNodeByRadius(pcl::PointXYZ searchPoint, float radius)
{
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  std::vector<pcl::PointXYZ> pvNode;

  if( g_kdTree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
      pvNode.push_back(g_pTree->points[pointIdxRadiusSearch[i]]);
    }
  }
  return pvNode;
}

void magneticVectorForce(const double* init, const double* target, double *B)
{
  double ln = 60.0; 	// Length of conductors
  double d = 1.61/2.0;		// Distance between the conductors m & n
  int m = 1; 		// Direction of current through the conductor, m(Right) 1(INTO), -1(OUT)
  int n = -1; 		// Direction of current through the conductor, n(Left) 1(INTO), -1(OUT)
  int N = 12; 		// Number sections/elements in the conductors
  int dl = ln/N; 		// Length of each element

  double xxP = target[0];
  double zzP = target[1];

  double q[3]={init[0], init[1], init[2]-RADIANS(90.0)};
  //Eigen::MatrixXd (1,1);

  MatrixXd Rot(2,2);
  Rot << cos(q[2]), -sin(q[2]),
      sin(q[2]),  cos(q[2]);

  MatrixXd Cm = Vector2d(q[0], q[1])+Rot*Vector2d(d/2.0,0); // Cm_ --> (2,1)
  MatrixXd Cn = Vector2d(q[0], q[1])+Rot*Vector2d(-d/2.0,0); // Cn_ --> (2,1)
  // cout << "Cm : " << Cm << endl;                             //
  // cout << "Cn : " << Cn << endl;

  MatrixXd xCm = Cm(0)*MatrixXd::Ones(1,N);
  MatrixXd xCn = Cn(0)*MatrixXd::Ones(1,N);

  // cout << "xCm : " << xCm <<endl;
  // cout << "xCn : " << xCn <<endl;

  // Y Coordinate of each element from origin, half on +Y & other half on -Y and also sam for both conductors
  double low = -ln/2.0+dl/2.0;
  double high = ln/2.0-dl/2.0;

  VectorXd yC = VectorXd::LinSpaced( ((high-low)/dl+1),low, low+dl*(N-1) );
  // cout << "yC : " << yC <<endl;

  // zC remains 0 throughout the length, as conductors are lying on XY plane
  MatrixXd zCm = Cm(1)*MatrixXd::Ones(1,N);
  MatrixXd zCn = Cn(1)*MatrixXd::Ones(1,N);
  // cout << "zCm : " << zCm << endl;
  // cout << "zCn : " << zCn << endl;

  // Length(Projection) 7 Direction of each current element in Vector form
  MatrixXd Lx = MatrixXd::Zero(1,N);	// Length of each element is zero on X axis
  MatrixXd Ly = dl*MatrixXd::Ones(1,N);	// Length of each element is dl on Y axis
  MatrixXd Lz = MatrixXd::Zero(1,N);	// Length of each element is zero on Z axis

  double Bx = 0;
  double By = 0;
  double Bz = 0;

  //#pragma omp parallel reduction(+:Bx, Bz)
  for( int i = 0; i < N; i++ )
  {
    double rxm = xxP - xCm(i); // Displacement Vector along X direction, from cond m..
    double rxn = xxP - xCn(i); // Displacement Vector along X direction, from cond n..

    double ry = yC(i);	// Same for m & n, no detector points on Y direction..

    double rzm = zzP - zCm(i); // Same for m & n..
    double rzn = zzP - zCn(i); // Same for m & n..

    double rm = sqrt(rxm*rxm + ry*ry + rzm*rzm);// Displacement Magnitude for an element on cond m..
    double rn = sqrt(rxn*rxn + ry*ry + rzn*rzn);// Displacement Magnitude for an element on cond n..

    double r3m = rm*rm*rm;
    double r3n = rn*rn*rn;

    Bx += + m*Ly(i)*rzm/r3m + n*Ly(i)*rzn/r3n;	// m & n, direction of current element..
    Bz += - m*Ly(i)*rxm/r3m - n*Ly(i)*rxn/r3n;	// m & n, direction of current element..
    // By = 0;
    //    cout << Bx <<" "<< Bz <<" " <<endl;
  }
  B[0] += Bx;
  B[1] += Bz;
}

double computeRepulsiveForce(double K, double dist, double range, double x, double tar_x)
{
  // cout <<"A "<< dist << " " << range <<" " << x << " " << tar_x << " " << K*((1.0f/dist)-(1.0f/range))*(1.0f/(dist*dist*dist))*(x-tar_x) << endl;

  if( dist <= range )
    return K*((1.0f/dist)-(1.0f/range))*(1.0f/(dist*dist))*(x-tar_x);
  else
    return 0;

}

double computeAttractiveForce(double K, double x, double tar_x)
{
  return -1.0f * K * (x - tar_x);
}


VectorXd ComputePotentialField2(double x, double y, double yaw, double* start, double* goal)
{
  Vector3d ret = Vector3d::Zero(3);

  Vector3d ran = Vector3d::Zero(3);
  ran(0) = cos(yaw);
  ran(1) = sin(yaw);

  std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),RANGE_OBS);
  double sumX=0,sumY=0;
  //#pragma omp parallel reduction(+:sumX, sumY)
  for( int i=0; i<obs.size(); i++)
  {
    double obs_x = obs[i].x;
    double obs_y = obs[i].y;

    if(obs_x == 0.0 && obs_y == 0.0)
      continue;

    double obs_dist = sqrt(((x - obs_x) * (x - obs_x)) + ((y - obs_y) * (y - obs_y)));

    sumX += computeRepulsiveForce(K_REP_OBS, obs_dist, RANGE_OBS, x, obs_x);
    sumY += computeRepulsiveForce(K_REP_OBS, obs_dist, RANGE_OBS, y, obs_y);
  }
  Vector3d pot = Vector3d::Zero(3);


  double rho = sqrt(sumX*sumX+sumY*sumY)*10.0;
  if( rho > 0 )
  {
    pot(0) = sumX;
    pot(1) = sumY;
    pot.normalize();

    Vector3d vDir = Vector3d::Zero(3);
    vDir = pot.cross(ran);

    {
      double yaw=0;
      if( vDir(2) > 0 )
      {
        yaw = boost::math::constants::pi<double>()*0.5;
      }
      else
      {
        yaw = -boost::math::constants::pi<double>()*0.5;
      }
      double pot_x =pot(0)*cos(yaw)-pot(1)*sin(yaw);
      double pot_y =pot(0)*sin(yaw)+pot(1)*cos(yaw);

      pot(0) = pot_x;
      pot(1) = pot_y;
    }

    if( rho > 1 ) rho = 1.0;
  }
  else    // ρ > ρmax
  {
    rho = 0;

    if( rand()%10 > 2 )
    {
      double start_dist = sqrt(((x - start[0]) * (x - start[0])) + ((y - start[1]) * (y - start[1])));
      double x_rep = computeRepulsiveForce(K_REP, start_dist, RANGE_REP, x, start[0]);
      double y_rep = computeRepulsiveForce(K_REP, start_dist, RANGE_REP, y, start[1]);

      // Vector Force (Unit Vector)
      ran(0) =sumX+(x_rep);
      ran(1) =sumY+(y_rep);

      double x_att = computeAttractiveForce(K_ATT, x, goal[0]);
      double y_att = computeAttractiveForce(K_ATT, y, goal[1]);

      ran(0) +=(x_att);
      ran(1) +=(y_att);

      ran.normalize();
    }
  }


  ran(0) =rho*pot(0)+(1-rho)*ran(0);
  ran(1) =rho*pot(1)+(1-rho)*ran(1);

  Vector3d mag = Vector3d::Zero(3);
  {
    double B[2]={0,0};
    double target[2]={x,y};
    magneticVectorForce(start, target, B);

    mag(0) += B[0]*5.0;
    mag(1) += B[1]*5.0;
  }
  {
    double B[2]={0,0};
    double target[2]={x,y};
    magneticVectorForce(goal, target, B);

    mag(0) += B[0]*5.0;
    mag(1) += B[1]*5.0;
  }
  double beta = sqrt(mag(0)*mag(0)+mag(1)*mag(1));
  if( beta > 0 )
  {
    mag(0) = mag(0)/beta;
    mag(1) = mag(1)/beta;
    if( beta > 1 ) beta = 1.0;
  }
  else
  {
    beta = 0;
  }
  ret(0) = beta*mag(0)+(1-beta)*ran(0);
  ret(1) = beta*mag(1)+(1-beta)*ran(1);
  return ret;
}

VectorXd ComputeObstacleField(double x, double y)
{
  VectorXd ret = VectorXd::Zero(5);

  ret(0) = x;
  ret(1) = y;

  std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),RANGE_OBS);

  double sumX=ret(2),sumY=ret(3);
  //#pragma omp parallel reduction(+:sumX, sumY)
  for( int i=0; i<obs.size(); i++)
  {
    double obs_x = obs[i].x;
    double obs_y = obs[i].y;

    if(obs_x == 0.0 && obs_y == 0.0)
      continue;

    double obs_dist = sqrt(((x - obs_x) * (x - obs_x))
                           + ((y - obs_y) * (y - obs_y)));

    sumX += computeRepulsiveForce(K_REP_OBS, obs_dist, RANGE_OBS, x, obs_x);
    sumY += computeRepulsiveForce(K_REP_OBS, obs_dist, RANGE_OBS, y, obs_y);
  }
  ret(2)=sumX;
  ret(3)=sumY;
  ret(4) = sqrt(ret(2)*ret(2)+ret(3)*ret(3));
  return ret;
}

bool isFreeSpace(float x, float y)
{
  bool isFreeSpace = true;

  std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),SAFEREGION);

  if( obs.size() > 0 )
    isFreeSpace = false;

  return isFreeSpace;
}

bool isFreeSpace_(float x, float y)
{
  bool isFreeSpace = true;

  std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),1.414);

  if( obs.size() > 0 )
    isFreeSpace = false;

  return isFreeSpace;
}

bool isValid(double x, double y, double yaw)
{
  double _x = x + 0.2*cos(yaw);
  double _y = y + 0.2*sin(yaw);

  bool isFreeSpace1 = isFreeSpace_(_x,_y);

  _x = x + 1.2*cos(yaw);
  _y = y + 1.2*sin(yaw);

  bool isFreeSpace2 = isFreeSpace_(_x,_y);

  _x = x + 2.2*cos(yaw);
  _y = y + 2.2*sin(yaw);

  bool isFreeSpace3 = isFreeSpace_(_x,_y);

  return isFreeSpace1 && isFreeSpace2 && isFreeSpace3;
}

bool DO( double* from_d, double* to_d, double* target_d, bool isLocal = false)
{
  //  return false;
  if( isLocal )
  {
    if( (to_d[0] == GOAL_L[0] && to_d[1] == GOAL_L[1]) )
    {
      return false;
    }
  }
  else
  {
    if( (to_d[0] == GOAL_G[0] && to_d[1] == GOAL_G[1]) )
    {
      return false;
    }
  }

  //    if( sqrt((from_d[0]-to_d[0])*(from_d[0]-to_d[0])+(from_d[1]-to_d[1])*(from_d[1]-to_d[1]))>0 )
  //    {
  //cout <<"out"<<endl;
  //       return false;
  //    }
  // else
  // {
  //  cout << "in"<<endl;
  // }

  double target_ori[3];
  target_ori[0] = target_d[0];
  target_ori[1] = target_d[1];
  target_ori[2] = target_d[2];

  int ITERMAX = 20;
  int iter = 0;

  Vector3d vGradient = Vector3d::Zero(3);
  Vector3d vMomentum = Vector3d::Zero(3);

  // Update Gradient
  VectorXd VectorField = ComputeObstacleField(target_d[0],target_d[1]);

  double w1= log(VectorField(4)+1)*0.5+0.001;

  if( VectorField(4) > 0 )
  {
    vGradient(0) = VectorField(2);
    vGradient(1) = VectorField(3);
    vGradient.normalize();

    vMomentum(0) = cos(target_d[2]);
    vMomentum(1) = sin(target_d[2]);

    Vector3d vDir = Vector3d::Zero(3);
    vDir = vMomentum.cross(vGradient);

    double vDirAng = vDir(2);
    if( fabs(vDir(2)) > RADIANS(90.0))
    {

      if( vDir(2) > 0 )
      {
        vDirAng = RADIANS(90.0);
      }
      else
      {
        vDirAng =-RADIANS(90.0);
      }

      double vFeasibleAngle = AngleUtils::toRange_PItoPI(target_d[2]+vDirAng);

      vGradient(0)  = cos(vFeasibleAngle);
      vGradient(1)  = sin(vFeasibleAngle);
    }
  }
  else
  {
#ifdef DRAW1
    ////////////////////////////////////////
    geometry_msgs::PoseStamped poseStamped;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
    header.frame_id = "/camera";
    poseStamped.pose.position.x = target_d[1];
    poseStamped.pose.position.y = -target_d[0];
    poseStamped.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
    header.frame_id = "/camera";
    poseStamped.pose.position.x = target_d[0];
    poseStamped.pose.position.y = target_d[1];
    poseStamped.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif
    poseStamped.header = header;



    poseStamped.pose.orientation = odom_quat;
    g_posArray2.poses.push_back(poseStamped.pose);
    ////////////////////////////////////////
#endif
    return true;
  }


  bool ret = false;
  while(iter < ITERMAX)
  {
    // Update Momentum
    double B[2]={0,0};
    magneticVectorForce(from_d, target_d, B);
    magneticVectorForce(to_d, target_d, B);


    double randValue = 0;
    int randSign = rand()%2;

    if( randSign != 1 )
    {
      randValue*=-1.0;
    }

    //randValue=0.0;
    target_d[2] =AngleUtils::toRange_PItoPI( atan2(B[1], B[0])+randValue);

    if( isValid(target_d[0], target_d[1], target_d[2]) )
    {
      ret = true;
      break;
    }
    target_d[0] += w1*vGradient(0);
    target_d[1] += w1*vGradient(1);

#ifdef DRAW
    ////////////////////////////////////////
    geometry_msgs::PoseStamped poseStamped;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
    header.frame_id = "/camera";
    poseStamped.pose.position.x = target_d[1];
    poseStamped.pose.position.y = -target_d[0];
    poseStamped.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
    header.frame_id = "/camera";
    poseStamped.pose.position.x = target_d[0];
    poseStamped.pose.position.y = target_d[1];
    poseStamped.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif

    poseStamped.header = header;



    poseStamped.pose.orientation = odom_quat;
    g_posArray3.poses.push_back(poseStamped.pose);
    ////////////////////////////////////////
#endif
    iter++;
  }

  if( ret )
  {
#ifdef DRAW
    ////////////////////////////////////////
    geometry_msgs::PoseStamped poseStamped;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
    header.frame_id = "/camera";
    poseStamped.pose.position.x = target_d[1];
    poseStamped.pose.position.y = -target_d[0];
    poseStamped.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
    header.frame_id = "/camera";
    poseStamped.pose.position.x = target_d[0];
    poseStamped.pose.position.y = target_d[1];
    poseStamped.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif
    poseStamped.header = header;


    poseStamped.pose.orientation = odom_quat;
    g_posArray2.poses.push_back(poseStamped.pose);
    ////////////////////////////////////////
#endif
  }
  return ret;
}

bool DesiredOrientation(ob::State* from, ob::State* to, ob::State* target)
{
  STATETYPE *from_ = from->as<STATETYPE>();
  STATETYPE *to_ = to->as<STATETYPE>();
  STATETYPE *target_ = target->as<STATETYPE>();

  double from_d[3]; from_d[0]=from_->getX(); from_d[1]=from_->getY(); from_d[2]=from_->getYaw();
  double to_d[3]; to_d[0]=to_->getX(); to_d[1]=to_->getY(); to_d[2]=to_->getYaw();
  double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();

  if( DO(from_d, to_d, target_d) )
  {
    target_->setX(target_d[0]);
    target_->setY(target_d[1]);
    target_->setYaw(target_d[2]);
    return true;
  }
  return false;
}

VectorXd magneticfield(ob::State* target, ob::State* nouse1, ob::State* nouse2) //rstate, NULL, NULL
{
  //    return VectorXd::Zero(2);
  STATETYPE *target_ = target->as<STATETYPE>();
  double target_d[3];
  target_d[0]=target_->getX();
  target_d[1]=target_->getY();
  target_d[2]=target_->getYaw();

  if( (target_d[0] == GOAL_G[0] && target_d[1] == GOAL_G[1]) )
  {
#ifdef DRAW
    {
      geometry_msgs::PoseStamped poseStamped;

      std_msgs::Header header;
      header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
      header.frame_id = "/camera";
      poseStamped.pose.position.x = target_d[1];
      poseStamped.pose.position.y = -target_d[0];
      poseStamped.pose.position.z = 0.0;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
      header.frame_id = "/camera";
      poseStamped.pose.position.x = target_d[0];
      poseStamped.pose.position.y = target_d[1];
      poseStamped.pose.position.z = 0.0;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif

      poseStamped.header =header;


      poseStamped.pose.orientation = odom_quat;
      g_posArray1.poses.push_back(poseStamped.pose);
    }
#endif
    return VectorXd::Zero(2);
  }
  else
  {
    double target_ori[3];
    target_ori[0] = target_d[0];
    target_ori[1] = target_d[1];
    target_ori[2] = target_d[2];
    VectorXd VectorField;
    if( nouse1 == NULL )
    {
      VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],START_G,GOAL_G);
    }
    else
    {
      double NEAR[3];
      STATETYPE *nouse1_ = nouse1->as<STATETYPE>();
      NEAR[0]=nouse1_->getX();
      NEAR[1]=nouse1_->getY();
      NEAR[2]=nouse1_->getYaw();
      VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],NEAR,GOAL_G);
    }
    target_d[2] =AngleUtils::toRange_PItoPI( atan2(VectorField(1), VectorField(0)));

    target_->setYaw(target_d[2]);
#ifdef CONFIG_ANALYSIS
    cout << target_d[0] << "\t"<<target_d[1] << "\t"<<target_d[2] << "\t"<<endl;
#endif
#ifdef DRAW

    {
      geometry_msgs::PoseStamped poseStamped;

      std_msgs::Header header;
      header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
      header.frame_id = "/camera";
      poseStamped.pose.position.x = target_d[1];
      poseStamped.pose.position.y = -target_d[0];
      poseStamped.pose.position.z = 0.0;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
      header.frame_id = "/camera";
      poseStamped.pose.position.x = target_d[0];
      poseStamped.pose.position.y = target_d[1];
      poseStamped.pose.position.z = 0.0;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif


      poseStamped.header =header;


      poseStamped.pose.orientation = odom_quat;
      g_posArray1.poses.push_back(poseStamped.pose);
    }
#endif
    return VectorXd::Zero(2);
  }
  return VectorXd::Zero(2);
}

bool randomCheck(ob::State* rand)
{
  STATETYPE *rand_ = rand->as<STATETYPE>();

  double x = rand_->getX();
  double y =rand_->getY();

  return !isFreeSpace(x,y);

}

bool isStateValid(const ob::SpaceInformation *si, const vector<vector<VectorXd> >& map, const ob::State *state)
{
  const STATETYPE *s = state->as<STATETYPE>();
  return si->satisfiesBounds(s) && isValid(s->getX(),s->getY(),s->getYaw());
}

// publish data for visualization
void VisualizePath() {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "/camera";

  nav_msgs::Path msg;
  msg.header = header;

  g_posArray1.header = header;
  g_msgpub1.publish(g_posArray1);
  g_posArray1.poses.clear();

  g_posArray2.header = header;
  g_msgpub2.publish(g_posArray2);
  g_posArray2.poses.clear();

  g_posArray3.header = header;
  g_msgpub3.publish(g_posArray3);
  g_posArray3.poses.clear();

  for(int i=0;i<vPath_g.size(); i++)
  {
    double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

    geometry_msgs::PoseStamped waypt;
    waypt.header = header;

    waypt.pose.position.x = x;
    waypt.pose.position.y = y;
    waypt.pose.position.z = 0;

    msg.poses.push_back(waypt);
  }
  // publish to /Path_RRT
  pub_path_rrt.publish(msg);
}

void UpdateGlobalPathData()
{
  if (g_solved)
  {
    og::PathGeometric path= ss_g->getSolutionPath();
    path.interpolate(100);

    vPath_g.clear();
    if( g_pTree_Path != NULL )
      g_pTree_Path->clear();

    g_pTree_Path  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i=0;i<path.getStateCount(); i++)
    {
      const STATETYPE *s = path.getState(i)->as<STATETYPE>();
      double x=s->getX(), y=s->getY(), yaw=s->getYaw();

      g_pTree_Path->push_back(pcl::PointXYZ(x, y, 0.0));
      vPath_g.push_back(Vector3d(x,y,yaw));
    }

    g_kdTree_Path.setInputCloud(g_pTree_Path);

    VisualizePath();
  }
}

// Planning Code
void plan_init(og::SimpleSetup* ss,double* start, double* goal)
{
  ob::ScopedState<> ss_start(g_space), ss_goal(g_space);
  ////////////////////////////////
  // set the start and goal states
  ss_start[0] =start[0];
  ss_start[1] =start[1];
  ss_start[2] =start[2];
  ss_goal[0] = goal[0];
  ss_goal[1] = goal[1];
  ss_goal[2] = goal[2];

  ss->setStartAndGoalStates(ss_start, ss_goal, RESOLUTION);

  // set state validity checking for this space
  ob::SpaceInformationPtr si(ss->getSpaceInformation());
  ss->setStateValidityChecker(std::bind(&isStateValid, si.get(),g_map, std::placeholders::_1));

  // DO-RRTstar
  if( b_DORRT_STAR != 0 )
  {
    cout <<"[+] DORRT MODE"<<endl;
    ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation,true,false,true,"DORRTstar"));
  }
  // RRTstar
  else
  {
    cout <<"[+] RRT MODE"<<endl;
    og::RRTstar *RRTstar = new og::RRTstar(ss->getSpaceInformation());
    RRTstar->setRange(5.0);
    ss->setPlanner(ob::PlannerPtr(RRTstar));
  }

  // this call is optional, but we put it in to get more output information
  ss->getSpaceInformation()->setStateValidityCheckingResolution(RESOLUTION);
  ss->setup();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void plan(bool b_goal)
{
  if(!b_goal) return;

  cout << "BOUNDS: " << BOUNDS.low[0] << ", " <<BOUNDS.low[1] << ", "<<BOUNDS.high[0] <<", " <<BOUNDS.high[1]<< endl;  // ed: DEBUG
  cout << "START_G: " << START_G[0] << ", " <<START_G[1] << ", "<<START_G[2] << endl;  // ed: DEBUG
  cout << "GOAL_G: " << GOAL_G[0] << ", " << GOAL_G[1] << ", " << GOAL_G[2] << endl;  // ed: DEBUG
  g_space->as<STATESPACE>()->setBounds(BOUNDS);
  plan_init(ss_g, START_G, GOAL_G);

  g_solved = ss_g->solve(PLANNINGTIME);

  cout << "g_sovled: " << g_solved << endl;

  UpdateGlobalPathData();

  b_goal = false;
}

// /points_obstacle_registered callback function
void points_obstacle_registered_callback(const VPointCloud::ConstPtr& msg,
                                         tf::TransformListener *listener,
                                         tf::StampedTransform *transform_)
{
  try {
    listener->lookupTransform("odom", "camera", ros::Time(0), *transform_);
  }
  catch (tf::TransformException ex) {
    ROS_WARN("%s", ex.what());
  }
  tf::Transform inv_transform = transform_->inverse();

  vector<Vector2d> vObstacle;

  if(!vObstacle.empty()) vObstacle.clear();

  for(auto it=msg->points.begin(); it != msg->points.end(); it++) {
    tf::Vector3 pt = inv_transform * tf::Vector3(it->x, it->y, 0);
    vObstacle.push_back(Vector2d(pt.getX(), pt.getY()));
  }

  // custom blocking (C-shape)
  if(b_block) {
    cout << "[+] custom blocking..." << endl;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(block_pos_x, block_pos_y,0));
    transform.setRotation(block_quat);

    for(int x=-25; x<20; x++) {
      for(int y=-20; y<20; y++) {
        tf::Vector3 pt;
        if(0.1*y > 1.25 || 0.1*y < -1.25) {
          tf::Vector3 vp = transform * tf::Vector3(0.1*x, 0.1*y, 0);
          pt = inv_transform * tf::Vector3(vp.getX(), vp.getY(), 0);

          vObstacle.push_back(Vector2d(pt.getX(), pt.getY()));
        }
        else if(0.1*x < -2) {
          tf::Vector3 vp = transform * tf::Vector3(0.1*x, 0.1*y, 0);
          pt = inv_transform * tf::Vector3(vp.getX(), vp.getY(), 0);

          vObstacle.push_back(Vector2d(pt.getX(), pt.getY()));
        }
      }
    }
    b_block=false;
  }


  if( g_pTree != NULL )
    g_pTree->clear();

  g_pTree  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  for(int k=0; k<vObstacle.size(); k++)
  {
    g_pTree->push_back(pcl::PointXYZ(vObstacle[k](0), vObstacle[k](1), 0.0));
  }

  g_kdTree.setInputCloud(g_pTree);
}

// /target_parking_space callback function
void target_parking_space_callback(const geometry_msgs::PoseStamped::ConstPtr& msg,
                                   tf::TransformListener *listener,
                                   tf::StampedTransform *transform_)
{
  try {
    listener->lookupTransform("odom", "camera", ros::Time(0), *transform_);
  }
  catch (tf::TransformException ex) {
    ROS_WARN("%s", ex.what());
  }

  tf::Transform inv_transform_ = transform_->inverse();

  tf::Vector3 goal = inv_transform_ * tf::Vector3(msg->pose.position.x,
                                                  msg->pose.position.y,
                                                  0);

  GOAL_G[0] = goal.getX();
  GOAL_G[1] = goal.getY();
  GOAL_G[2] = tf::getYaw(msg->pose.orientation);
  b_goal = true;

  // for custom blocking (C-shape)
  block_pos_x = msg->pose.position.x;
  block_pos_y = msg->pose.position.y;
  block_quat.setValue(msg->pose.orientation.x,
                      msg->pose.orientation.y,
                      msg->pose.orientation.z,
                      msg->pose.orientation.w);

  b_block = true;

  plan(b_goal);
}

int main(int argc, char* argv[]) {
  ss_g = new og::SimpleSetup(g_space);
  ss_l = new og::SimpleSetup(g_space_local);

  ros::init(argc, argv, "decision_maker_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  tf::TransformListener listener;
  tf::StampedTransform transform;

  BOUNDS.low[0] = -50.0;
  BOUNDS.low[1] = -50.0;
  BOUNDS.high[0] = 50.0;
  BOUNDS.high[1] = 50.0;

  g_msgpub1 = nh.advertise<geometry_msgs::PoseArray>("PoseArray_RRT_1", 1);
  g_msgpub2 = nh.advertise<geometry_msgs::PoseArray>("PoseArray_RRT_2", 1);
  g_msgpub3 = nh.advertise<geometry_msgs::PoseArray>("PoseArray_RRT_3", 1);

  pub_path_rrt = nh.advertise<nav_msgs::Path>("Path_RRT", 1);

  sub_potential_array = nh.subscribe<VPointCloud>("points_obstacle_registered", 1, boost::bind(&points_obstacle_registered_callback,_1,&listener, &transform));
  sub_target_parking_space = nh.subscribe<geometry_msgs::PoseStamped>("target_parking_space", 1, boost::bind(&target_parking_space_callback, _1, &listener, &transform));

  cout << "[+] decision maker node has started..."<<endl;

  ros::spin();

  return 0;
}
