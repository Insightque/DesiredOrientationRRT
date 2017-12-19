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
#include <boost/program_options.hpp>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

// The easy problem is the standard narrow passage problem: two big open
// spaces connected by a narrow passage. The hard problem is essentially
// one long narrow passage with the robot facing towards the long walls
// in both the start and goal configurations.

bool isStateValid(const ob::SpaceInformation *si, const ob::State *state)
{
    const ob::ClothoidStateSpace::StateType *s = state->as<ob::ClothoidStateSpace::StateType>();
//    double x=s->getX(), y=s->getY();
 
    return si->satisfiesBounds(s); //&& fabs(s->getK()) < 2.0;// && (x<40|| x>60 || (y>10.0 && y<12.0));
}

void plan(ob::StateSpacePtr space,double time, bool isClothoid)
{
	ob::ScopedState<> start(space), goal(space);
    ob::RealVectorBounds bounds(2);
    bounds.low[0] = 0;
    bounds.high[0] = 100.0;
	
    bounds.low[1] = 0;
    bounds.high[1] = 50.0;

	//bounds.setLow(0);
    //bounds.setHigh(100);
    
    space->as<ob::ClothoidStateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ob::SpaceInformationPtr si(ss.getSpaceInformation());
    ss.setStateValidityChecker(std::bind( &isStateValid, si.get(), std::placeholders::_1));

    // set the start and goal states
	start[0] =85.0;
	start[1] = 8.0; 
	start[2] = 0.99*boost::math::constants::pi<double>();
	goal[0] = 20.0;
	goal[1] = 24; 
	goal[2] = 0.99*boost::math::constants::pi<double>();
	if( isClothoid )
	{
		start[3] = 0.0;
		goal[3] = 0.0;
	}
    
	ss.setStartAndGoalStates(start, goal);

    ss.setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
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
