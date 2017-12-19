#include "ompl/control/planners/rrt/RG-RRT.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = false;
    siC_ = si.get();
    addIntermediateStates_ = false;
    lastGoalMotion_ = NULL;

    goalBias_ = 0.10;

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates);

    const std::vector<double> diff = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().getDifference();
    for(double d : diff)
        control_offset.push_back(d / double(this->RSIZE));
}

ompl::control::RGRRT::~RGRRT(void)
{
    freeMemory();
}

void ompl::control::RGRRT::setup(void)
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
    nn_->setDistanceFunction(std::bind(&RGRRT::distanceFunction, this,
        std::placeholders::_1, std::placeholders::_2));

}

void ompl::control::RGRRT::clear(void)
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = NULL;
}

void ompl::control::RGRRT::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0; i < motions.size(); ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            if (motions[i]->control)
                siC_->freeControl(motions[i]->control);
            delete motions[i];
        }
    }
}

void ompl::control::RGRRT::setupReachableSet(Motion* const m)
{
    const std::vector<double>& low_bound = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().low;
    const std::vector<double>& high_bound = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds().high;
    for(int i = 0; i < this->RSIZE; ++i)
    {
        Motion* motion = new Motion(siC_);
        siC_->copyControl(motion->control, m->control);
        double*& controls = motion->control->as<RealVectorControlSpace::ControlType>()->values;
        controls[0] = low_bound[0] + control_offset[0] * (i+1);
        for(int j = 1; j < low_bound.size(); ++j)
            controls[j] = high_bound[j]/2;

        motion->steps = siC_->propagateWhileValid(m->state, motion->control, siC_->getMinControlDuration(), motion->state);

        if(motion->steps != 0)
            m->reachable.push_back(motion); 
    }
}

int ompl::control::RGRRT::selectReachableMotion(const Motion* qnear, const Motion* qrand)
{
    const double nearD = si_->distance(qnear->state, qrand->state);
    double minD = nearD;
    const std::vector<Motion*>& reachable = qnear->reachable;
    int id = -1;
    for(int i = 0; i < reachable.size(); ++i)
    {
        double newD = si_->distance(reachable[i]->state, qrand->state);
        //std::cout << "newD: " << newD << " nearD: " << nearD << std::endl;
        if(newD < minD)
        {
            minD = newD;
            id = i;
        }
    }
    return id;
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        setupReachableSet(motion);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nn_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();

    Motion      *rmotion = new Motion(siC_);
    base::State  *rstate = rmotion->state;
    Control       *rctrl = rmotion->control;
    base::State  *xstate = si_->allocState();

    while (ptc == false)
    {
        // Reject samples until there exists a sample where qr in R(qnear) is closer to the sample than qnear
        int id = -1;
        Motion *nmotion = NULL;
        base::State *state = NULL;
        Control *ctrl = NULL;
        unsigned int cd = 0;
       for(int i = 0; i < 100 && id == -1; ++i) 
        {
            /* sample random state (with goal biasing) */
            if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
                goal_s->sampleGoal(rstate);
            else
                sampler_->sampleUniform(rstate);

            /* find closest state in the tree */
            nmotion = nn_->nearest(rmotion);

            // Select a sample from the nearest state's reachable set that is closer to the random state
            id = selectReachableMotion(nmotion, rmotion);
        }

        // Uncomment for better performance with Pendulum
        if(id != -1)
        {
               state = nmotion->reachable[id]->state;
               ctrl = nmotion->reachable[id]->control;
               cd = nmotion->reachable[id]->steps;
        }
        else
        {
        ctrl = siC_->allocControl();
        state = si_->allocState(); 
        si_->copyState(state, rstate);
        cd = controlSampler_->sampleTo(ctrl, nmotion->control, nmotion->state, state);
        }

        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, ctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for ( ; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    Motion *motion = new Motion();
                    motion->state = pstates[p];
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, ctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    setupReachableSet(motion);
                    nn_->add(motion);
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
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

                //free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (size_t p = 0 ; p < pstates.size(); ++p)
                    si_->freeState(pstates[p]);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* create a motion */
                Motion *motion = new Motion(siC_);
                si_->copyState(motion->state, state);
                siC_->copyControl(motion->control, ctrl);
                motion->steps = cd;
                motion->parent = nmotion;
                setupReachableSet(motion);

                if(id == -1)
                {
                    si_->freeState(state);
                    siC_->freeControl(ctrl);
                }

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
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
    }

    bool solved = false;
    bool approximate = false;
#ifdef APPROXIMATE
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }
#endif
    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        PathControl *path = new PathControl(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        const Motion* m = motions[i];
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                        base::PlannerDataVertex(m->state),
                        control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state),
                        base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
