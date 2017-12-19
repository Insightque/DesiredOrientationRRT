/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Mark Moll */

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ClothoidStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/DORRTstar.h>
#include <boost/program_options.hpp>
#include <Eigen/Core> 

using namespace std;
using namespace Eigen;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

void magneticVectorForce(double x, double y, double theta,
        VectorXd& ret)
{
    /*
	double ln = 60.0; 	// Length of conductors
	double d = 1.61/2.0;		// Distance between the conductors m & n
	int m = 1; 		// Direction of current through the conductor, m(Right) 1(INTO), -1(OUT)
	int n = -1; 		// Direction of current through the conductor, n(Left) 1(INTO), -1(OUT)
	int N = 12; 		// Number sections/elements in the conductors
	int dl = ln/N; 		// Length of each element
	
	// XYZ coordinates/Location of each element from the origin (0,0,0), i.e 'd/2' is taken as origin..
    MatrixXd xCm = (d/2.0)*MatrixXd::Ones(1,N); 
    MatrixXd xCn = (-d/2.0)*MatrixXd::Ones(1,N);
	
	// Y Coordinate of each element from origin, half on +Y & other half on -Y and also sam for both conductors
	VectorXd yC = VectorXd::LinSpaced(N, -ln/2.0+dl/2.0, ln/2.0-dl/2.0);

	// zC remains 0 throughout the length, as conductors are lying on XY plane
	MatrixXd zC = MatrixXd::Zero(1,N);
	
	// Length(Projection) 7 Direction of each current element in Vector form
	MatrixXd Lx = MatrixXd::Zero(1,N);	// Length of each element is zero on X axis
	MatrixXd Ly = dl*MatrixXd::Ones(1,N);	// Length of each element is dl on Y axis
	MatrixXd Lz = MatrixXd::Zero(1,N);	// Length of each element is zero on Z axis
		
	// Points/Locations in space (here XZ plane) where B is to be computed
	int NP = 96;	//Detector points..
	int xPmax = 12;	//Dimensions of detector space.., arbitrary..
	int zPmax = 12;

	MatrixXd resVec_x = MatrixXd::Zero(NP,NP);
	MatrixXd resVec_y = MatrixXd::Zero(NP,NP);
	MatrixXd resPos_x = MatrixXd::Zero(NP,NP);
	MatrixXd resPos_y = MatrixXd::Zero(NP,NP);
	
	//Divide space with NP points..
	RowVectorXd xP = RowVectorXd::LinSpaced(NP, -xPmax, xPmax);	
	VectorXd zP = VectorXd::LinSpaced(NP, -zPmax, zPmax);

	// Creating the Mesh..
	MatrixXd xxP = xP.replicate(NP,1); 
	MatrixXd zzP = zP.replicate(1,NP);

	//Initialize B..
	MatrixXd Bx = MatrixXd::Zero(NP,NP);
	MatrixXd By = MatrixXd::Zero(NP,NP);
	MatrixXd Bz = MatrixXd::Zero(NP,NP);

	// Computation of Magnetic Field (B) using Superposition principle..
	// Compute B at each detector points due to each small cond elements & integrate them..
	for( int q=0; q<N; q++ )
	{
		MatrixXd rxm = xxP.array() - xCm(0,q); // Displacement Vector along X direction, from cond m..
		MatrixXd rxn = xxP.array() - xCn(0,q); // Displacement Vector along X direction, from cond n..
		
		double ry = yC(q,0);	// Same for m & n, no detector points on Y direction..
		
		MatrixXd rz = zzP.array() - zC(0,q); // Same for m & n..
		
		MatrixXd rm = (rxm.array()*rxm.array()+ry*ry+rz.array()*rz.array()).sqrt();	// Displacement Magnitude for an element on cond m..
		MatrixXd rn = (rxn.array()*rxn.array()+ry*ry+rz.array()*rz.array()).sqrt();	// Displacement Magnitude for an element on cond n..
		
		MatrixXd r3m = rm.array()*rm.array()*rm.array();
		MatrixXd r3n = rn.array()*rn.array()*rn.array();
		
		Bx = Bx.array() + m*Ly(0,q)*rz.array()/r3m.array() + n*Ly(0,q)*rz.array()/r3n.array();	// m & n, direction of current element..
		// By = 0;
		Bz = Bz.array() - m*Ly(0,q)*rxm.array()/r3m.array() - n*Ly(0,q)*rxn.array()/r3n.array();
	}
		
	MatrixXd Rot(2,2);
	Rot << cos(_theta), -sin(_theta),
	       sin(_theta),  cos(_theta);

	MatrixXd tmpMat(2, NP);
	for(int i=0; i<NP; i++)
	{
		//tmpMat << Bx.row(i)+xxP.row(i),
		//		Bz.row(i)+zzP.row(i);
		tmpMat << Bx.row(i),
				Bz.row(i);
		tmpMat = Rot * tmpMat;
		
		// Block Operation
		resVec_x.block(i,0,1,NP)  = tmpMat.block(0,0,1,NP).array();
		resVec_y.block(i,0,1,NP)  = tmpMat.block(1,0,1,NP).array();
		
		tmpMat << xxP.row(i),
				zzP.row(i);
		tmpMat = Rot * tmpMat;
		
		// Block Operation
		resPos_x.block(i,0,1,NP)  = (tmpMat.block(0,0,1,NP)).array()+_x;
		resPos_y.block(i,0,1,NP)  = (tmpMat.block(1,0,1,NP)).array()+_y;
	}

	///////////////////////////////////////////////////////////////////
	geometry_msgs::PoseArray poseArray;
	
    double grid_offset=grid_dim_/2.0*m_per_cell_;
    double K_rep_mag = 100.0f; 
    
	for(int i=0; i<NP; i++)
	{
		for( int j=0; j<NP; j++)
		{
			int i_ = int((resPos_x(i,j) + grid_offset - m_per_cell_/2.0) / m_per_cell_);
			int j_ = int((resPos_y(i,j) + grid_offset - m_per_cell_/2.0) / m_per_cell_);
			
			if( i_ >= 0 && j_ >= 0 && i_ < NP && j_ < NP )
			{
				PotentialMap[i_][j_].bInit = true;
				
				double vSize = sqrtf(resVec_x(i,j)*resVec_x(i,j) + resVec_y(i,j)*resVec_y(i,j));
				double vDist = sqrtf((_x-resPos_x(i,j))*(_x-resPos_x(i,j)) + (_y-resPos_y(i,j))*(_y-resPos_y(i,j)));
				if( vDist < 0.1 ) vDist = 0.1;
				PotentialMap[i_][j_].F_sum.x += K_rep_mag*resVec_x(i,j)/vSize/vDist;
				PotentialMap[i_][j_].F_sum.y += K_rep_mag*resVec_y(i,j)/vSize/vDist;
			}
		}
	}
	*/
}
double computeRepulsiveForce(double K, double dist, double range, double x, double tar_x)
{
    // cout <<"A "<< dist << " " << range <<" " << x << " " << tar_x << " " << K*((1.0f/dist)-(1.0f/range))*(1.0f/(dist*dist*dist))*(x-tar_x) << endl;

    return K*((1.0f/dist)-(1.0f/range))*(1.0f/(dist*dist*dist))*(x-tar_x);

}

double computeAttractiveForce(double K, double x, double tar_x)
{
    return -1.0f * K * (x - tar_x);
}
/** Basic unit-norm rotation field. */
VectorXd vectorfield(const ob::State *from,const ob::State *to)
{
//    const ob::RealVectorStateSpace::StateType &x = *state->as<ob::RealVectorStateSpace::StateType>();
    VectorXd v(2);
  //  v[0] = x[1];
   // v[1] = -x[0];
   // v.normalize();
    return v;
}

/** Basic unit-norm rotation field. */
VectorXd magneticfield(const ob::State *from,const ob::State *to)
{
    //const ob::RealVectorStateSpace::StateType &x = *state->as<ob::RealVectorStateSpace::StateType>();
    VectorXd v(2);
   // v[0] = x[1];
   // v[1] = -x[0];
   // v.normalize();
    return v;
}

// The easy problem is the standard narrow passage problem: two big open
// spaces connected by a narrow passage. The hard problem is essentially
// one long narrow passage with the robot facing towards the long walls
// in both the start and goal configurations.

bool isStateValid(const ob::SpaceInformation *si, const ob::State *state)
{
    const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    double x=s->getX(), y=s->getY(), yaw=s->getYaw();

	// 1. Collision Checking
	// CollisionChecking(x, y);

	//bool isFreeSpace = (x<40|| x>60 || (y>10.0 && y<12.0));
	bool isFreeSpace = (x<40|| x>60 || (y>10.8 && y<11.2));
    return si->satisfiesBounds(s) && isFreeSpace;
}

bool isStateValid_Clothoid(const ob::SpaceInformation *si, const ob::State *state)
{
    const ob::ClothoidStateSpace::StateType *s = state->as<ob::ClothoidStateSpace::StateType>();
    double x=s->getX(), y=s->getY(), yaw=s->getYaw();

	// 1. Collision Checking
	// CollisionChecking(x, y);

	//bool isFreeSpace = (x<40|| x>60 || (y>10.0 && y<12.0));
	bool isFreeSpace = (x<40|| x>60 || (y>10.8 && y<11.2));
    return si->satisfiesBounds(s) && isFreeSpace;
}

void plan(ob::StateSpacePtr space,double time, bool isClothoid)
{


    // 0. Build KdTree
	// =BuildTree()

	ob::ScopedState<> start(space), goal(space);
    ob::RealVectorBounds bounds(2);
    bounds.low[0] = 0;
    bounds.high[0] = 100.0;
	
    bounds.low[1] = 0;
    bounds.high[1] = 50.0;

	//bounds.setLow(0);
    //bounds.setHigh(100);
    
	if( isClothoid )
    	space->as<ob::ClothoidStateSpace>()->setBounds(bounds);
	else
    	space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ob::SpaceInformationPtr si(ss.getSpaceInformation());
    
	ss.setStateValidityChecker(std::bind(
        isClothoid ? &isStateValid_Clothoid : &isStateValid, si.get(), std::placeholders::_1));

    // set the start and goal states
	start[0] =85.0;
	start[1] = 8.0; 
	start[2] = 0.99*boost::math::constants::pi<double>();
	goal[0] = 50.0;
	goal[1] = 11.0; 
	//goal[2] = 0.99*boost::math::constants::pi<double>();
	goal[2] = 0.0*boost::math::constants::pi<double>();
	if( isClothoid )
	{
		start[3] = 0.0;
		goal[3] = 0.0;
	}
    
	ss.setStartAndGoalStates(start, goal);

    ///////////////////////////////////////////////////////////////////
    // 1. Potential Field

    double m_per_cell_ = 0.25;
    int grid_dim_x = ceil((bounds.high[0] - bounds.low[0])/m_per_cell_);
    int grid_dim_y = ceil((bounds.high[1] - bounds.low[1])/m_per_cell_);
	
    double num_obs[grid_dim_x][grid_dim_y];
    
    for (int x = 0; x < grid_dim_x; x++) {
        for (int y = 0; y < grid_dim_y; y++) {
            num_obs[x][y]=0;
        }
    }
    
    vector<Vector2d> obs;
    for (unsigned int i=0; i<obs.size(); i++)
    {
        int x = ((grid_dim_x/2)+obs[i](0)/m_per_cell_);
        int y = ((grid_dim_y/2)+obs[i](1)/m_per_cell_);
        if( x >= 0 && x < grid_dim_x && y >= 0 && y < grid_dim_y )
        {
            num_obs[x][y]++;
        }
    }
    
    vector<vector<VectorXd> > VectorMap;
	// Initialize Potential Map
    for (int x = 0; x < grid_dim_x; x++)
    {
		vector<VectorXd> v;
		VectorMap.push_back(v);
		
        for (int y = 0; y < grid_dim_y; y++)
        {
            VectorXd vec = VectorXd::Zero(3);
            VectorMap[x].push_back(vec);
        }
    }

    // Parameters
    double grid_offset_x=grid_dim_x/2.0*m_per_cell_;
    double grid_offset_y=grid_dim_y/2.0*m_per_cell_;
    double Range_obs = 10.0;  
    double K_rep_obs = 1.3f;

    double Range_rep=180.0;
    double K_rep = 50.0;
    
    double K_att = 30.0;
	
    for (int i = 0; i<grid_dim_x; i++)
    {
        for (int j = 0; j<grid_dim_y; j++)
        {
            // transform index to local coordinate (x,y)
            double x = -grid_offset_x + (i*m_per_cell_+m_per_cell_/2.0);
            double y = -grid_offset_y + (j*m_per_cell_+m_per_cell_/2.0);

            // Generate obstacle potential
            for (unsigned int k = 0; k <= obs.size(); k++)
            {
                double obs_x = obs[k](0);
                double obs_y = obs[k](1);

                if(obs_x == 0.0 && obs_y == 0.0)
                    continue;

                double obs_dist = sqrt(((x - obs_x) * (x - obs_x))
                        + ((y - obs_y) * (y - obs_y)));

                if (obs_dist > 0.0 && obs_dist < Range_obs)
                {
                    VectorMap[i][j](2) = 1;
                    VectorMap[i][j](0) += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, x, obs_x);
                    VectorMap[i][j](1) += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, y, obs_y);
                }
            }

            // Summation of attractive and repulsive potential values
            if( VectorMap[i][j](2) == 1)
            {	

                double x_att = computeAttractiveForce(K_att, x, goal[0]);
                double y_att = computeAttractiveForce(K_att, y, goal[1]);

                double start_dist = sqrt(((x - start[0]) * (x - start[0]))
                        + ((y - start[1]) * (y - start[1])));
                double x_rep = computeRepulsiveForce(K_rep, start_dist,
                        Range_rep, x, start[0]);
                double y_rep = computeRepulsiveForce(K_rep, start_dist,
                        Range_rep, y, start[1]);

                // Vector Force (Unit Vector)
                VectorMap[i][j](0) +=(x_att + x_rep);
                VectorMap[i][j](1) +=(y_att + y_rep);
            }

        }
    }
    //////////////////////////////////////////////////////////////////

//    ss.setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
    ss.setPlanner(std::make_shared<ompl::geometric::DORRTstar>(ss.getSpaceInformation()
                ,vectorfield, magneticfield));
	//b, l::base::PlannerPtr(new ompl::geometric::KPIECE1(ss.getiSpaceInformation())), range);
    // this call is optional, but we put it in to get more output information
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss.setup();
    ss.print();

    // attempt to solve the problem within 30 seconds of planning time
    ob::PlannerStatus solved = ss.solve(time);

    if (solved)
    {
        std::vector<double> reals;

        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        og::PathGeometric path = ss.getSolutionPath();
        path.interpolate(1000);
        path.printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

void printTrajectory(ob::StateSpacePtr space, const std::vector<double>& pt)
{
    if (pt.size()!=3) throw ompl::Exception("3 arguments required for trajectory option");
    const unsigned int num_pts = 50;
    ob::ScopedState<> from(space), to(space), s(space);
    std::vector<double> reals;

    from[0] = from[1] = from[2] = 0.;

    to[0] = pt[0];
    to[1] = pt[1];
    to[2] = pt[2];

    std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";
    for (unsigned int i=0; i<=num_pts; ++i)
    {
        space->interpolate(from(), to(), (double)i/num_pts, s());
        reals = s.reals();
        std::cout << "path " << reals[0] << ' ' << reals[1] << ' ' << reals[2] << ' ' << std::endl;
    }
}

void printDistanceGrid(ob::StateSpacePtr space)
{
    // print the distance for (x,y,theta) for all points in a 3D grid in SE(2)
    // over [-5,5) x [-5, 5) x [-pi,pi).
    //
    // The output should be redirected to a file, say, distance.txt. This
    // can then be read and plotted in Matlab like so:
    //     x = reshape(load('distance.txt'),200,200,200);
    //     for i=1:200,
    //         contourf(squeeze(x(i,:,:)),30);
    //         axis equal; axis tight; colorbar; pause;
    //     end;
    const unsigned int num_pts = 200;
    ob::ScopedState<> from(space), to(space);
    from[0] = from[1] = from[2] = 0.;

    for (unsigned int i=0; i<num_pts; ++i)
        for (unsigned int j=0; j<num_pts; ++j)
            for (unsigned int k=0; k<num_pts; ++k)
            {
                to[0] = 5. * (2. * (double)i/num_pts - 1.);
                to[1] = 5. * (2. * (double)j/num_pts - 1.);
                to[2] = boost::math::constants::pi<double>() * (2. * (double)k/num_pts - 1.);
                std::cout << space->distance(from(), to()) << '\n';
            }

}

int main(int argc, char* argv[])
{
    try
    {
        po::options_description desc("Options");
        desc.add_options()
            ("help", "show help message")
            ("clothoid", "use Clothoid state space")
            ("clothoidrev", "use reverse Clothoid state space")
            ("dubins", "use Dubins state space")
            ("dubinssym", "use symmetrized Dubins state space")
            ("reedsshepp", "use Reeds-Shepp state space (default)")
            ("time", po::value<double>(),"plannig time")
            ("trajectory", po::value<std::vector<double > >()->multitoken(),
                "print trajectory from (0,0,0) to a user-specified x, y, and theta")
            ("distance", "print distance grid")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
            po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        if (vm.count("help") || argc==1)
        {
            std::cout << desc << "\n";
            return 1;
        }

        ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(5.88));

		bool isClothoid = false;

        if (vm.count("clothoidrev"))
		{
            space = ob::StateSpacePtr(new ob::ClothoidStateSpace(0.17, true));
			isClothoid = true;
		}
		
		if (vm.count("clothoid"))
		{
			space = ob::StateSpacePtr(new ob::ClothoidStateSpace(0.17, false));
			isClothoid = true;
		}
		
		if (vm.count("dubins"))
            space = ob::StateSpacePtr(new ob::DubinsStateSpace(5.88, false));
        if (vm.count("dubinssym"))
            space = ob::StateSpacePtr(new ob::DubinsStateSpace(5.88, true));
    
		double time = 30.0;

		if (vm.count("time"))
			time = vm["time"].as<double>();

		plan(space,time,isClothoid);

        if (vm.count("trajectory"))
            printTrajectory(space, vm["trajectory"].as<std::vector<double> >());
        if (vm.count("distance"))
            printDistanceGrid(space);
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
