/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "ompl/geometric/planners/rrt/DORRT_2.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalState.h"

#include <sys/time.h>
ompl::geometric::DORRT_2::DORRT_2(const base::SpaceInformationPtr &si, const RandomCheck &rc, const MagneticField &mf, const DesiredOrientation &gdo):
    base::Planner(si, "DORRT_NO-NN_NO-GDO"),
    rc_(rc),
    mf_(mf),
    gdo_(gdo)
{
    specs_.approximateSolutions = false;
    specs_.directed = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    lastGoalMotion_ = nullptr;

    Planner::declareParam<double>("range", this, &DORRT_2::setRange, &DORRT_2::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &DORRT_2::setGoalBias, &DORRT_2::getGoalBias, "0.:.05:1.");
}

ompl::geometric::DORRT_2::~DORRT_2()
{
    freeMemory();
}

void ompl::geometric::DORRT_2::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::DORRT_2::setup()
{
//
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
//    maxDistance_ = 8.0;
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    nn_->setDistanceFunction(std::bind(&DORRT_2::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
}

void ompl::geometric::DORRT_2::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::DORRT_2::solve_time(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution  = nullptr;
    Motion *approxsol = nullptr;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    struct timeval start_point, end_point;
    struct timeval start_point_nn, end_point_nn;
    struct timeval start_point_add, end_point_add;
    struct timeval start_point_rng, end_point_rng;
    struct timeval start_point_tot, end_point_tot;
    double operating_time;
    double operating_time_suc;
    double operating_time_tot;
    double operating_time_nn;
    double operating_time_add;
    double operating_time_rng;

    int chkmotion=0;
    int chkmotion_suc=0;
    gettimeofday(&start_point_tot, NULL);
    while (ptc == false)
    {

        gettimeofday(&start_point_rng, NULL);
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        bool bPotSmp = rc_(rstate);  
        if(bPotSmp)
        {
            gettimeofday(&end_point_rng, NULL);
            operating_time_rng += (double)(end_point_rng.tv_sec)+(double)(end_point_rng.tv_usec)/1000000.0-(double)(start_point_rng.tv_sec)-(double)(start_point_rng.tv_usec)/1000000.0;
            // collision is true
            continue;
        }
        else
        {
            gettimeofday(&end_point_rng, NULL);
            operating_time_rng += (double)(end_point_rng.tv_sec)+(double)(end_point_rng.tv_usec)/1000000.0-(double)(start_point_rng.tv_sec)-(double)(start_point_rng.tv_usec)/1000000.0;
            mf_(rstate, NULL, NULL); 
        }

        gettimeofday(&start_point_nn, NULL);
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        gettimeofday(&end_point_nn, NULL);
        operating_time_nn += (double)(end_point_nn.tv_sec)+(double)(end_point_nn.tv_usec)/1000000.0-(double)(start_point_nn.tv_sec)-(double)(start_point_nn.tv_usec)/1000000.0;
        /* find state to add */
        gettimeofday(&start_point, NULL);
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }
/*        if( gdo_(nmotion->state, rstate, dstate) )
        {
            double d = si_->distance(nmotion->state, dstate);

            if (d > maxDistance_)
            {
                si_->getStateSpace()->interpolate(nmotion->state, dstate, maxDistance_ / d, xstate);
                dstate = xstate;
            }
        }
        */
        bool bCheckMotion = si_->checkMotion(nmotion->state, dstate);
        gettimeofday(&end_point, NULL);
        operating_time += (double)(end_point.tv_sec)+(double)(end_point.tv_usec)/1000000.0-(double)(start_point.tv_sec)-(double)(start_point.tv_usec)/1000000.0;
        chkmotion++;
        if (bCheckMotion)
        {
            chkmotion_suc++;
            operating_time_suc += (double)(end_point.tv_sec)+(double)(end_point.tv_usec)/1000000.0-(double)(start_point.tv_sec)-(double)(start_point.tv_usec)/1000000.0;
        
        gettimeofday(&start_point_add, NULL);
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        gettimeofday(&end_point_add, NULL);
        operating_time_add += (double)(end_point_add.tv_sec)+(double)(end_point_add.tv_usec)/1000000.0-(double)(start_point_add.tv_sec)-(double)(start_point_add.tv_usec)/1000000.0;
        }
    }

    gettimeofday(&end_point_tot, NULL);
    printf("motion: %d motion_suc:%d\n",chkmotion,chkmotion_suc);
    operating_time_tot = (double)(end_point_tot.tv_sec)+(double)(end_point_tot.tv_usec)/1000000.0-(double)(start_point_tot.tv_sec)-(double)(start_point_tot.tv_usec)/1000000.0;
    printf("ext_ratio: %f,rng: %f,nn: %f,ext: %f,add: %f\n",operating_time_suc/operating_time*100,operating_time_rng/operating_time_tot*100,operating_time_nn/operating_time_tot*100,operating_time/operating_time_tot*100,operating_time_add/operating_time_tot*100);
    printf("t_rng: %f,t_nn: %f,t_ext: %f,t_add: %f,t_tot:%f\n",operating_time_rng,operating_time_nn,operating_time,operating_time_add,operating_time_tot);


    bool solved = false;
    bool approximate = false;

#ifdef APPROXIMATE
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }
#endif
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}
ompl::base::PlannerStatus ompl::geometric::DORRT_2::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution  = nullptr;
    Motion *approxsol = nullptr;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {

        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        
        if( rc_(rstate) ) // collision is true
            continue;
        mf_(rstate, NULL, NULL); 
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }
/*
        if( gdo_(nmotion->state, rstate, dstate) )
        {
            double d = si_->distance(nmotion->state, dstate);
            si_->getStateSpace()->interpolate(nmotion->state, dstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }
  */      
        if (si_->checkMotion(nmotion->state, dstate))
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.0;
            bool sat = goal->isSatisfied(motion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = motion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;

#ifdef APPROXIMATE
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }
#endif
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::DORRT_2::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
}
