#include <iostream>
//#include "../inc/ParkingPlannerThread.h"
#include <ros/ros.h>    //ROS湲곕낯 ?ㅻ뜑 ?뚯씪
#include <cmath>
#include "../inc/quadtree.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeometricUtils.h>
#include <MathParam.h>
#include <AngleUtils.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ClothoidStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <boost/program_options.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/timer/timer.hpp>
#include <boost/thread/thread.hpp>
#include<ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <std_msgs/Float32MultiArray.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <fstream>
#define DRAW
//#define DRAWVEHICLE
//#define CONFIG_ANALYSIS

//#define ODEDEMO
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;
namespace po = boost::program_options;
using namespace Eigen;


//bool CLOTHOID = true;
//ob::StateSpacePtr space(new ob::ClothoidStateSpace(0.17, true)); // false: forward
//typedef ob::ClothoidStateSpace::StateType STATETYPE;
//typedef ob::ClothoidStateSpace STATESPACE;

int MODEDORRTSTAR = 1;
og::SimpleSetup* ss_g;
og::SimpleSetup* ss_l;
vector<Vector3d> vPath_g;

vector<Vector3d> vTrajec_g;

ofstream outFile("output.txt");
bool CLOTHOID = false;
//ob::StateSpacePtr g_space(new ob::DubinsStateSpace(4.88, true)); // false: forward
//ob::StateSpacePtr g_space_local(new ob::DubinsStateSpace(4.88, true)); // false: forward
ob::StateSpacePtr g_space(new ob::DubinsStateSpace(6, true)); // false: forward
ob::StateSpacePtr g_space_local(new ob::DubinsStateSpace(6, true)); // false: forward
typedef ob::SE2StateSpace::StateType STATETYPE;
typedef ob::SE2StateSpace STATESPACE;
///////////////////////////////////////////////////////////////////////////////////////////////////////////
string LOGFILENAME="DEFAULT";
bool UPDATINGMAP = false;
int TMPMOVINGCNT=0;
int TMPMOVINGPOSCNT=0;
int ITER = 1;
int LOGFLAG=0;
double PLANNINGTIME = 5.0;
double PLANNINGTIME_L = 1.0;
double COSTTHRESHOLD=0.0;

vector<Vector2d> OBSTACLE;
double START_GORI[3]={0.0,0.0,RADIANS(0)};
double GOAL_GORI[3]={0.0,0.0,RADIANS(0)};
double START_G[3]={0.0,0.0,RADIANS(0)};
double GOAL_G[3]={0.0,0.0,RADIANS(0)};
//For local planner
double START_L[3]={0.0,0.0,RADIANS(0)};
double GOAL_L[3]={0.0,0.0,RADIANS(0)};
ob::RealVectorBounds BOUNDS(2);
double SAFEREGION = 0.6;

double RANGE_OBS = 4.0;  
double K_REP_OBS = 1.00f;
double RANGE_REP=180.0;
double K_REP = 100.0;    
double K_ATT = 0.002;

//double CAR_C2F = 3.2; //ori
//double CAR_C2R = 0.8; //ori
double CAR_C2F = 2.843;
double CAR_C2R = 1.435;

double CAR_WIDTH = 0.9;

vector<vector<VectorXd> > g_map; //unuse
ob::PlannerStatus g_solved;
bool g_replan = false;
bool g_solved_init = false;
double g_localization_map[4]={0.0,0.0,RADIANS(0),0.0};
double g_localization[4]={0.0,0.0,RADIANS(0),0.0};
double VEHICLEPOS_TMP[4]={0,0,0,0};
// KD TREE : Obstacle
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree;
pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree;

// KD TREE : Path
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree_Path;
pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree_Path;

// Publish Msg
ros::Publisher g_msgpub1;
ros::Publisher g_msgpub2;
ros::Publisher g_msgpub3;
ros::Publisher g_msgpub4;
ros::Publisher g_msgpub5;

ros::Subscriber g_msgsub1;
//ros::Subscriber g_msgsub2;

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

bool isFreeSpace_(float x, float y)
{
    bool isFreeSpace = true;

    std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),SAFEREGION);

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


bool isStateValid(const ob::SpaceInformation *si, const vector<vector<VectorXd> >& map, const ob::State *state)
{
    const STATETYPE *s = state->as<STATETYPE>();
    return si->satisfiesBounds(s) && isValid(s->getX(),s->getY(),s->getYaw());
}

void UpdateGlobalPathData()
{
    if (g_solved && g_solved_init)
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
            double x=s->getX(), y=s->getY(), yaw=s->getYaw();// 경로에서 x,y,h을 뽑아와서 저 아래 벡터에 넣는다.

            g_pTree_Path->push_back(pcl::PointXYZ(x, y, 0.0));
            vPath_g.push_back(Vector3d(x,y,yaw));// 이게 글로벌 경로임
        }

        g_kdTree_Path.setInputCloud(g_pTree_Path);// 장애물 각각에서 충돌이 없는 경로를 하기보다는 만든 경로에서 일정 거리에 장애물이 있는지 없는지 알아보는 것임

    }
}

void MergeLocal2GlobalPathData(int startIdx, int goalIdx, og::PathGeometric* path_l)// 지역적으로 만든 경로를 기존에 전역 경로에 붙인다. 이전에 없앤 글로벌한 경로의 개수만큼만.
{
    if (g_solved && g_solved_init)
    {
        for(int i=0;i<goalIdx-startIdx+1; i++)
        {
            const STATETYPE *s = path_l->getState(i)->as<STATETYPE>();
            double x=s->getX(), y=s->getY(), yaw=s->getYaw();

            vPath_g[i+startIdx]=Vector3d(x,y,yaw);
            g_pTree_Path->points[i+startIdx] = pcl::PointXYZ(x, y, 0.0);
        }
        g_kdTree_Path.setInputCloud(g_pTree_Path);
    }
}

void plan_init(og::SimpleSetup* ss,double* start, double* goal, bool isLocal = false)
{
    if( isLocal )
    {
        ob::ScopedState<> ss_start(g_space_local), ss_goal(g_space_local);
        ////////////////////////////////
        // set the start and goal states
        ss_start[0] =start[0];
        ss_start[1] =start[1]; 
        ss_start[2] =start[2];
        ss_goal[0] = goal[0];
        ss_goal[1] = goal[1]; 
        ss_goal[2] = goal[2];

        ss->setStartAndGoalStates(ss_start, ss_goal,0.05);
    }
    else
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

        ss->setStartAndGoalStates(ss_start, ss_goal,0.05);
    }
    // set state validity checking for this space
    ob::SpaceInformationPtr si(ss->getSpaceInformation());
    ss->setStateValidityChecker(std::bind(
                &isStateValid, si.get(),
                g_map, std::placeholders::_1));

    if( isLocal )
    {
        //ss->setPlanner(ob::PlannerPtr(new og::CForest(ss->getSpaceInformation())));
        ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation()));
    }
    else
    {
        ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation()));
    }

    // this call is optional, but we put it in to get more output information
    ss->getSpaceInformation()->setStateValidityCheckingResolution(0.05);
    ss->setup();
}
    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool plan(og::SimpleSetup* ss, double time, bool isLocal = false)
{
    //    ss.print();
    // attempt to solve the problem within 30 seconds of planning time
    if( isLocal )
    {
        cout <<"### LOCAL ### "<<time<<endl;
        return ss->solve(time);
    }
    else
    {
        //cout <<"### GLOBAL ### "<<time<<endl;
        g_solved = ss->solve(time);
        g_solved_init = true;

        return g_solved;
    }
}

void planner_global()
{
    // Global Path Planning
    // Update Global Information
    //////////////////////////////////////////////////////
    START_G[0] = g_localization_map[0];//전역 맵에서의 차량의 현재 글로벌한 x,y,h를 받는다.
    START_G[1] = g_localization_map[1];
    START_G[2] = g_localization_map[2];


    g_space->as<STATESPACE>()->setBounds(BOUNDS);// 바운더리 값은 입력으로 미리 받는것이다.
   
    cout << START_G[0] << " " <<START_G[1] <<" "<<START_G[2]<<" "<<GOAL_G[0] <<" "<<GOAL_G[1] << " "<<GOAL_G[2]<<endl;

    plan_init(ss_g,START_G,GOAL_G, false);
    
    plan(ss_g,PLANNINGTIME);
    UpdateGlobalPathData();
}

// 0 : No Global Path Data
// -1: Fail to Find NN
// -2: Fail to Goal Index ( collision )
// -3: Fail to Local Path Generation ( Cannot solve )
// 1: Success to Local Path
int planner_local()
{
    if (g_solved && g_solved_init  )
    {
        if( g_replan == false )
        {
            return 2;
        }
        cout <<"PATH REPLANNING - LOCAL"<<endl;
        // Find Nearest Node Index from the global path w.r.t current pos (g_localization_map)

        double goal[3]={0,0,0};
        double start[3]={0,0,0};

        //START_G - global map 
        // 기준좌표계: x, y, theta, 대상좌표: _x, _y
        //GeometricUtils::TransRelativeCoord(double ref_x, double ref_y, double ref_h, double _x, double _y, double _h, double& ret_x, double& ret_y, double& ret_h)
        //GeometricUtils::TransRelativeCoord(START_G[0], START_G[1], START_G[2], g_localization_map[0], g_localization_map[1], g_localization_map[2], start[0], start[1], start[2]);
       
        start[0] = g_localization_map[0];
        start[1] = g_localization_map[1];
        start[2] = g_localization_map[2];

        int startIndex  =  SearchNearestNodeIdxByRadius(pcl::PointXYZ(start[0], start[1], 0), 10.0);

        if( startIndex == -1 )
        {
            cout <<"local -1" <<endl;
            return -1;  
        }

        int goalIndex = -1;
        int minGoalIndex = 40;
        int maxGoalIndex = 80;

        for(int i=startIndex+minGoalIndex; i<vPath_g.size();i++)
        {
            if( i > maxGoalIndex + startIndex )
            {
                break;
            }

            double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

            std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),0.5);
            if( obs.size() == 0 )
            {
                goal[0] = x;
                goal[1] = y;
                goal[2] = yaw;
                goalIndex = i;
                break;
            
            }
            else
            {
            }

        }

        cout <<"local : startidx["<<startIndex<<"] goalidx["<<goalIndex<<"]" <<endl;

        if(goalIndex == -1)
        {
            cout <<"local -2 : startidx["<<startIndex<<"] goalidx["<<goalIndex<<"]" <<endl;
            return -2; 
        }

        //cout <<"LOCAL "<<start[0] <<" "<< start[1]<<" " <<start[2] <<" "<<goal[0] <<" "<<goal[1] <<" "<< goal[2] <<endl;


        // For magnetic field, obstacle field
        START_L[0]=start[0];
        START_L[1]=start[1];
        START_L[2]=start[2];
        GOAL_L[0]=goal[0];
        GOAL_L[1]=goal[1];
        GOAL_L[2]=goal[2];

        vector<double> vX;
        vector<double> vY;
        double bound = 20.0;

        for(int i=startIndex;i<=goalIndex; i++)
        {
            double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

            vX.push_back(x+bound);
            vX.push_back(x-bound);
            vY.push_back(y+bound);
            vY.push_back(y-bound);
        }

        ob::RealVectorBounds bounds(2);

        bounds.low[0] = *min_element(vX.begin(),vX.end());
        bounds.high[0] =*max_element(vX.begin(),vX.end());
        bounds.low[1] = *min_element(vY.begin(),vY.end());
        bounds.high[1] =*max_element(vY.begin(),vY.end());

        g_space_local->as<STATESPACE>()->setBounds(bounds);
        
        plan_init(ss_l,start, goal, true);
        ///////////////////////////////////////////////////
        bool retplan = plan(ss_l, PLANNINGTIME_L, true);
        int ret = -3;
        if( retplan )
        {
            og::PathGeometric path = ss_l->getSolutionPath();
            path.interpolate(goalIndex - startIndex + 1);
            
            MergeLocal2GlobalPathData(startIndex, goalIndex, &path);
            ret = 1;
        }
        

        cout <<"local :"<<ret <<endl;
        cout <<"HH"<<endl;
        return ret;
    }
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// From HeightMap Module : Local Path Update : 5hz
void SubTopicProcess1(const std_msgs::Float32MultiArray::ConstPtr& msg) 
// 하이트맵에서 이 것을 받는 순간의 차량의 전역 x,y,h,어쩌고, 정보(4개)와 장애물의 정보(x1,y1,x2,y2, ...)가 들어온다. 포인트 클라우드는 특정 라이다 데이터가 특정 그리드 위에 하나라도 있으면 그 그리드의 가운데 점을 주는 것이고 애초에 특정 높이로 자르고 난 뒤에 저런 정보는 받는 것이다.
{
    UPDATINGMAP=true;
    // Global Coordinate Obstacle Data and Position Data
    vector<Vector2d> vObstacle;
    vObstacle.push_back(Vector2d(-99999.0,-99999.0));

    for(int i=0; i<int((msg->data.size()-4)/2.0); i++)//4개는 하이트맵 받는 순간의 차량의 전역 정보 4개이고 저걸 빼고 2로 나누면 총 장애물이 있는 그리드의 개수이다.
    {
	//vObstacle.push_back(Vector2d(msg->data.at(i*2+4),msg->data.at(i*2+5)));
    }
    cout <<"SUBDATA"<<endl;
    
    if( g_pTree != NULL )
        g_pTree->clear();
    
    g_pTree  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(int k=0; k<vObstacle.size(); k++)
    {
        //g_pTree->push_back(pcl::PointXYZ(vObstacle[k](0), vObstacle[k](1), 0.0)); //우선, 장애물 정보를 충돌 검사를 하기 위해서 g_pTree에 넣어 놓는다.
    }
    g_pTree->push_back(pcl::PointXYZ(100, 100, 0.0));  //  임의로 장애물 하나만 넣기. 아예 없으면 오류. 장애물 고려 안하려고 사용
    g_kdTree.setInputCloud(g_pTree);// 여기까지가 저 kdTree를 이용하기 위한 기본 세팅이다.

    /////////////////////////////////////////////////////////////////////////////
    // localization data (SLAM)
    g_localization_map[0] = msg->data.at(0);//하이트맵 받을 때의 차량 중심에서의 글로벌 x위치
    g_localization_map[1] = msg->data.at(1);
    g_localization_map[2] = msg->data.at(2);
    g_localization_map[2] = AngleUtils::toRange_PItoPI(g_localization_map[2]);
    g_localization_map[3] = msg->data.at(3);

    // 경로를 만드는 곳에서, 처음 로컬라이제이션 정보를 얻고 난 다음, 차량의 후륜 중심이 따라가는 경로를 만드는데, 이 시작점이 애초에
    // 차량 중심부터 시작한다. 즉, 경로의 시작점 위치가 차량 중심이면서, 이 순간 바로 차량이 후륜 중심이 있기를 바라는 아이러니에 빠지게 된다.
    // 따라서, 경로를 만들 때, 차량 후륜 중심부터 경로가 생길 수 있도록 경로를 생성하는 부분에서 처리를 해줘야 한다.
    double t_x = g_localization_map[0];
    double t_y = g_localization_map[1];
    double t_h = g_localization_map[2];

    //g_localization_map[0] = t_x - 1.745*cos( g_localization_map[2] );
    //g_localization_map[1] = t_y - 1.745*sin( g_localization_map[2] );

    //g_localization_map[0] = t_x - 0.8*cos( g_localization_map[2] ); //경로가 그려지는 지점과 저 0.8 수치를 같게 하면, Rviz에서 동일한 곳에서 경로를 만들 수 있네
    //g_localization_map[1] = t_y - 0.8*sin( g_localization_map[2] );
    g_localization_map[0] = t_x - CAR_C2R*cos( g_localization_map[2] ); //경로가 그려지는 지점과 저 0.8 수치를 같게 하면, Rviz에서 동일한 곳에서 경로를 만들 수 있네
    g_localization_map[1] = t_y - CAR_C2R*sin( g_localization_map[2] );
    
    g_replan = false;// 우선 처음 리플레닝이 바로 하진 않으니 우선은 초기 값으로 false
    for(int i=0; i<vPath_g.size(); i++) //우선, 전체 경로 사이즈 동안 전체 경로를 저 서치노드바이레디어스로 전역 경로 생성된 것이 있으면 그 경로 주변 0.2m안에 장애물이 있는지 확인
    {
        double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

        std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),0.2);
        if( obs.size() > 0 )// 장애물이 하나라도 지금의 전역 경로에 있으면 바로 경로 재 생성
        {
            g_replan = true;
            break;//for문 나가지겠지.
        }
    }


    if ( g_solved_init == false )//맨 처음에는 당연히 false겠네
    {
        g_replan = true;
    }

    int iterLocal = 0;
    int retLocal = planner_local();
    if( retLocal == -3 ) // -3인 케이스가 있다. 아예 경로를 만들 수 없을 때.
    {
        while( iterLocal < 5 )
        {
            retLocal = planner_local();
            if( retLocal > 1 )//경로 못 만들었다가 만들었으면 와일문에서 나가자.
                break;
            iterLocal++;
        }
    }
    if( retLocal < 1 )
    {
        cout <<"PATH REPLANNING - GLOBAL"<<endl;
        planner_global();
    }
    else
    {
        //cout << "path size["<<vPath_g.size()<<"] "<<msg->data.at(0) << " "<<msg->data.at(1) <<endl;
        //cout <<"PATH Tracking ........"<<endl;
    }
    
    /////////////////////////////////////////////////////////////////////
    // Pub local planner path to controller ( Global Coordinate )
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
        header.frame_id = "/camera_init_global";
#else
        header.frame_id = "/camera_init";
#endif
        nav_msgs::Path msg;
        msg.header = header;

        nav_msgs::Path msg2;
        msg2.header = header;

        g_posArray1.header = header;
        g_msgpub1.publish(g_posArray1);
        g_posArray1.poses.clear();

        g_posArray2.header = header;
        g_msgpub2.publish(g_posArray2);
        g_posArray2.poses.clear();

        g_posArray3.header = header;
        g_msgpub3.publish(g_posArray3);
        g_posArray3.poses.clear();

        for(int i=0;i<vPath_g.size(); i++)// vPath_g 에 전역 경로가 들어 있는 것이다.
        {

            double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

            geometry_msgs::PoseStamped waypt;
            waypt.header = header;
#ifdef DRAWVEHICLE
            waypt.pose.position.x = y;
            waypt.pose.position.y = -x;
            waypt.pose.position.z = 0;
#else
            waypt.pose.position.x = x;
            waypt.pose.position.y = y;
            waypt.pose.position.z = 0;
#endif
            msg.poses.push_back(waypt);
            
            geometry_msgs::PoseStamped waypt2;
            waypt2.header = header;
            waypt2.pose.position.x = x;
            waypt2.pose.position.y = y;
            waypt2.pose.position.z = 0;
            msg2.poses.push_back(waypt2);
        }

        g_msgpub5.publish(msg2); // 최종으로 경로가 토픽으로 퍼블리쉬 되는 곳
        g_msgpub4.publish(msg);
    }

    UPDATINGMAP=false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PLANNER_FRAMEWORK()
{
    int argc=0;
    char** argv;
    ros::init(argc, argv, "DecisionMaker");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");
    g_msgpub1 = node.advertise<geometry_msgs::PoseArray>("PoseArray_RRT_1", 1);//안쓴다.
    g_msgpub2 = node.advertise<geometry_msgs::PoseArray>("PoseArray_RRT_2", 1);
    g_msgpub3 = node.advertise<geometry_msgs::PoseArray>("PoseArray_RRT_3", 1);
    g_msgpub4 = node.advertise<nav_msgs::Path>("Path_RRT", 1);
    g_msgpub5 = node.advertise<nav_msgs::Path>("RNDFPathData", 1);
    
    g_msgsub1 = node.subscribe("velodyne_potential_array", 1, SubTopicProcess1);
    //g_msgsub2 = node.subscribe("velodyne_obstaclesGlobal", 1, SubTopicProcess2);
    
    cout << "START of DO-RRT*: ROS Version"<<endl;
    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        ros::spinOnce();
        sleep(0.01);
    }
    cout << "END of DO-RRT*: ROS Version"<<endl;
}

int main(int argc, char* argv[])
{

    try
    {
        po::options_description desc("Options");
        desc.add_options()
            ("x", po::value<double>(),"goalX")
            ("y", po::value<double>(),"goalY")
            ("yaw", po::value<double>(),"goalYaw")
            ("globalT", po::value<double>(),"globalT")
            ("localT", po::value<double>(),"localT")
            ("bminx", po::value<double>(),"BminX")
            ("bminy", po::value<double>(),"BminY")
            ("bmaxx", po::value<double>(),"BmaxX")
            ("bmaxy", po::value<double>(),"BmaxY")
            ("rrt", po::value<double>(),"rrtmode")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
                    po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        double x, y, yaw; 
        if (vm.count("x"))
        {
            GOAL_G[0] = vm["x"].as<double>();
        }
        if (vm.count("y"))
        {
            GOAL_G[1] = vm["y"].as<double>();
        }
        if (vm.count("yaw"))
        {
            GOAL_G[2] = RADIANS(vm["yaw"].as<double>());
            GOAL_G[2] = AngleUtils::toRange_PItoPI(GOAL_G[2]);
        }
        if (vm.count("globalT"))
        {
            PLANNINGTIME = vm["globalT"].as<double>();
        }
        if (vm.count("localT"))
        {
            PLANNINGTIME_L = vm["localT"].as<double>();
        }
        if (vm.count("bminx"))
        {
            BOUNDS.low[0] = vm["bminx"].as<double>();
        }
        if (vm.count("bmaxx"))
        {
            BOUNDS.high[0] = vm["bmaxx"].as<double>();
        }
        if (vm.count("bminy"))
        {
            BOUNDS.low[1] = vm["bminy"].as<double>();
        }
        if (vm.count("bmaxy"))
        {
            BOUNDS.high[1] = vm["bmaxy"].as<double>();
        }
        if (vm.count("rrt"))
        {
            MODEDORRTSTAR = 0;
        }
        VEHICLEPOS_TMP[0]=85.0;
        VEHICLEPOS_TMP[1]=0.0;
        VEHICLEPOS_TMP[2]=RADIANS(179);
        cout <<"BOUND " <<BOUNDS.low[0]<<" "<<BOUNDS.high[0]<<" "<<BOUNDS.low[1]<<" "<<BOUNDS.high[1]<<endl;

        GOAL_GORI[0] = GOAL_G[0];// GOAL_GORI은 안쓰임
        GOAL_GORI[1] = GOAL_G[1];
        GOAL_GORI[2] = GOAL_G[2];
/*
        //GOAL_G[0] = -10.0;
        //GOAL_G[1] = 0.0;
        //GOAL_G[2] = RADIANS(179);

        BOUNDS.low[0] = bminx;
        BOUNDS.high[0] = bmaxy;

        BOUNDS.low[1] = bminy;
        BOUNDS.high[1] = bmaxy;
        */
        ss_g = new og::SimpleSetup(g_space);
        
        ss_l = new og::SimpleSetup(g_space_local);

        boost::thread t=boost::thread(boost::bind(&PLANNER_FRAMEWORK));//실제로 저 플래너 프레임워크가 쓰레드로 계속 도는 것이다.

        t.join();
    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}
