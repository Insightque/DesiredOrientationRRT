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

/* Author: Seho Shin */

#include "ompl/base/spaces/ClothoidStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <boost/math/constants/constants.hpp>
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>

using namespace std;
using namespace ompl::base;

namespace
{
    const double twopi = 2. * boost::math::constants::pi<double>();
    const double pi = boost::math::constants::pi<double>();

	const double CLOTHOID_ZERO = -1e-9;

	//[0~2pi]
	double conv2pi(double x){
		x = fmod(x,twopi);
		if (x < 0)
			x += twopi;
		return x;
	}

	//[-pi,pi]
	double convpi(double x){
		x = fmod(x + pi,twopi);
		if (x < 0)
			x += twopi;
		return x - pi;
	}
	inline double mod2pi(double x)
    {
        if (x<0 && x>CLOTHOID_ZERO) return 0;
        return x - twopi * floor(x / twopi);
    }
};

ompl::base::State* ompl::base::ClothoidStateSpace::allocState() const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::base::ClothoidStateSpace::freeState(State *state) const
{
    CompoundStateSpace::freeState(state);
}

void ompl::base::ClothoidStateSpace::registerProjections()
{
    class ClothoidDefaultProjection : public ProjectionEvaluator
    {
    public:

        ClothoidDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        virtual unsigned int getDimension() const
        {
            return 2;

        }

        virtual void defaultCellSizes()
        {
            cellSizes_.resize(2);
            bounds_ = space_->as<ClothoidStateSpace>()->getBounds();
            cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
            //cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / magic::PROJECTION_DIMENSION_SPLITS;
        }

        virtual void project(const State *state, EuclideanProjection &projection) const
        {
            memcpy(&projection(0),
					state->as<ClothoidStateSpace::StateType>()->as<RealVectorStateSpace::StateType>(0)->values,
					2* sizeof(double));
        }
    };

    registerDefaultProjection(ProjectionEvaluatorPtr(dynamic_cast<ProjectionEvaluator*>(new
					ClothoidDefaultProjection(this))));
}
double ompl::base::ClothoidStateSpace::distance(const State *state1, const State *state2) const
{
	double W0 = weight_[0];
	double W1 = weight_[1];
	double W2 = weight_[2];

	const StateType *s1 = static_cast<const StateType*>(state1);
    const StateType *s2 = static_cast<const StateType*>(state2);
    double x1 = s1->getX(), y1 = s1->getY(), th1 = s1->getYaw();
    double x2 = s2->getX(), y2 = s2->getY(), th2 = s2->getYaw();
	
	
	if( x1 == x2 && y1 == y2 && th1 == th2 )
		return 0.;

    if( !CheckRange(s1, s2) )
        return 99999.0;
	
    if (isReversable_)
	{
		ClothoidPath pathForward = clothoid(state1, state2);
		ClothoidPath pathReverse = clothoid(state2, state1);

		//When searching in the forward direction, it should be compared with the
		//	curvature of the first node of the candidate clothoid.
		double clothoid_k = pathForward.k_;
		double clothoid_l = pathForward.length();

		double costF =
			W0*clothoid_l + W1*fabs(s1->getK()-clothoid_k);

		//When searching in the reverse direction, it should be compared with
		//the curvature of the last node of the candidate clothoid.
		clothoid_k = pathReverse.k_+pathReverse.dk_*pathReverse.l_;
		clothoid_l = pathReverse.length();

		double costR =
			W2*(W0*clothoid_l + W1*fabs(s1->getK()-clothoid_k));


		return std::min(costF, costR);
	}
	else
	{
		ClothoidPath path = clothoid(state1, state2);
		return W0*path.length()+W1*fabs(s1->getK()-path.k_);
	}
}

void ompl::base::ClothoidStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    bool firstTime = true;
	Clothoid::ClothoidCurve cc;
    ClothoidPath path(cc);
    interpolate(from, to, t, firstTime, path, state);
}

void ompl::base::ClothoidStateSpace::interpolate(const State *from, const State *to, const double t,
    bool &firstTime, ClothoidPath &path, State *state) const
{
    if (firstTime)
    {
        if (t>=1.)
        {
            if (to != state)
                copyState(state, to);
            return;
        }
        if (t<=0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }

        path = clothoid(from, to);
        if (isReversable_) // should be true in vehicle case - commented by shinsh
        {
			double W0 = weight_[0];
			double W1 = weight_[1];
			double W2 = weight_[2];
            ClothoidPath path2(clothoid(to, from));
   
			const StateType *s1 = static_cast<const StateType*>(from);

			double costF =
				W0*path.length()+W1*fabs(s1->getK()-path.k_);
			
            
		    double rev_k = path2.k_+path2.dk_*path2.l_;
            double costR =
				(W0*path2.length()+W1*fabs(s1->getK()-rev_k))*W2;

			if (costR < costF)
            {
                path2.reverse_ = true;
                path = path2;
            }
        }
        firstTime = false;
    }
	//const StateType *s1 = static_cast<const StateType*>(from);
	//const StateType *s2 = static_cast<const StateType*>(to);
	//cout << s1->getX() << " " << s1->getY() << " " <<s1->getYaw()<< endl;
	//cout << s2->getX() << " " << s2->getY() << " " <<s2->getYaw()<< endl;
    interpolate(from, path, t, state);
}

void ompl::base::ClothoidStateSpace::interpolate(const State *from, const
		ClothoidPath &path, double t, State *state) const
{
    StateType *s = allocState()->as<StateType>();
	double seg = t * path.length();

	Clothoid::valueType theta;
	Clothoid::valueType kappa;
	Clothoid::valueType x;
	Clothoid::valueType y;


    if (path.reverse_)
	{
		seg = path.length()-seg;
		path.cc_.eval(seg, theta, kappa, x, y);
	   // state->as<StateType>()->setYaw(conv2pi(conv2pi(theta)-pi));
    }
    else
    {
		path.cc_.eval(seg, theta, kappa, x, y);
    }
    state->as<StateType>()->setX(x);
    state->as<StateType>()->setY(y);
    getSubspace(1)->enforceBounds(s->as<SO2StateSpace::StateType>(1));
    state->as<StateType>()->setYaw(theta);
    state->as<StateType>()->setK(kappa);
    freeState(s);
	//cout << x << " " << y << " " <<theta << endl;
	//getchar();
}

ompl::base::ClothoidStateSpace::ClothoidPath
ompl::base::ClothoidStateSpace::clothoid(const State *state1, const State *state2) const
{

	Clothoid::ClothoidCurve cc;

    const StateType *s1 = static_cast<const StateType*>(state1);
    const StateType *s2 = static_cast<const StateType*>(state2);
    double x1 = s1->getX(), y1 = s1->getY(), th1 = convpi(s1->getYaw());
    double x2 = s2->getX(), y2 = s2->getY(), th2 = convpi(s2->getYaw());

//	cout << x1 << " " << y1 << " " << th1 << " " << x2 <<" " << y2 << " " <<
//		th2 << endl;
	cc.setup_G1(x1, y1, th1, x2, y2, th2);

	ompl::base::ClothoidStateSpace::ClothoidPath path(cc);

    return path;
}

bool ompl::base::ClothoidStateSpace::CheckRange(const State *state1, const
        State *state2) const
{
     const StateType *s1 = static_cast<const StateType*>(state1);
    const StateType *s2 = static_cast<const StateType*>(state2);
    double x1 = s1->getX(), y1 = s1->getY();
    double x2 = s2->getX(), y2 = s2->getY();
    
    if( sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) < 1.0 )
    {
        return false;
    }
    else
    {
        return true;
    }

}

void ompl::base::ClothoidMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<ClothoidStateSpace*>(si_->getStateSpace().get());
    if (!stateSpace_)
        throw Exception("No state space for motion validator");
}

bool ompl::base::ClothoidMotionValidator::checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const
{
   /* assume motion starts in a valid configuration so s1 is valid */

    if( !stateSpace_->CheckRange(s1, s2) )
        return false;
    
    bool result = true, firstTime = true;
	Clothoid::ClothoidCurve cc;
    ClothoidStateSpace::ClothoidPath path(cc);
    int nd = stateSpace_->validSegmentCount(s1, s2);

    nd = 30;
    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();

        for (int j = 1 ; j < nd ; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, firstTime, path, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first)
                    stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first)
                stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool ompl::base::ClothoidMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if( !stateSpace_->CheckRange(s1, s2) )
        return false;
    
   // if (!si_->isValid(s2))
   //     return false;
    
    bool result = true, firstTime = true;
    Clothoid::ClothoidCurve cc;
    ClothoidStateSpace::ClothoidPath path(cc);
    int nd = stateSpace_->validSegmentCount(s1, s2);

    nd = 30; 
    /* initialize the queue of test positions */
    std::queue< std::pair<int, int> > pos;
    if (nd >= 2)
    {
        pos.push(std::make_pair(1, nd - 1));

        /* temporary storage for the checked state */
        State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, path, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.push(std::make_pair(x.first, mid - 1));
            if (x.second > mid)
                pos.push(std::make_pair(mid + 1, x.second));
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
