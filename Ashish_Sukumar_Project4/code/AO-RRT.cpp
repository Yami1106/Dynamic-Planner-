///////////////////////////////////////
// RBE550
// Project 4
// Authors: Ashish Sukumar
//////////////////////////////////////

#include "AO-RRT.h"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/PathControl.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/PlannerData.h>

#include <limits>
#include <algorithm>
#include <iostream>

namespace ompl
{
    namespace control
    {
        // define class for AO-RRT
        AORRT::AORRT(const SpaceInformationPtr &si)
            : base::Planner(si, "AORRT")
        {
            // set planner to accept approximate solutions
            specs_.approximateSolutions = true;
            // set the space information pointer for control space
            siC_ = si.get();

            // initialize best cost bound to infinity
            bestCostBound_ = base::Cost(std::numeric_limits<double>::infinity());
            // initialize best path cost to infinity
            bestPathCost_  = bestCostBound_;

            // declare parameter for goal bias
            Planner::declareParam<double>(
                "goal_bias", this,&AORRT::setGoalBias, &AORRT::getGoalBias,"0.:.05:1.");
        }
        // destructor to free memory
        AORRT::~AORRT()
        {
            freeMemory();
        }
        // setup function
        void AORRT::setup()
        {
            base::Planner::setup();

            // nearest neighbor structure
            if (!nn_)
            // default to kd-tree
                nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
            // set distance function
            nn_->setDistanceFunction(
                [this](const Motion *a, const Motion *b)
                {
                    return distanceFunction(a, b);
                });

            // set optimization objective
            if (pdef_)
            {
                // check if optimization objective is set
                if (pdef_->hasOptimizationObjective())
                {
                    // get optimization objective from problem definition
                    opt_ = pdef_->getOptimizationObjective();
                }
                // if not set create a default one to path length
                else
                {
                    OMPL_WARN("%s: No optimization objective set. Using path length.",
                              getName().c_str());
                    opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
                    pdef_->setOptimizationObjective(opt_);
                }
            }

            // initialize best cost bound and best path cost
            bestCostBound_ = opt_->infiniteCost();
            bestPathCost_  = bestCostBound_;
        }

        // clear function
        void AORRT::clear()
        {
            // call the clear function of the base class
            Planner::clear();
            // reset samplers
            sampler_.reset();
            controlSampler_.reset();
            freeMemory();
            // clear nearest neighbor structure
            if (nn_)
                nn_->clear();

            bestPathStates_.clear();
            bestPathControls_.clear();
            bestPathSteps_.clear();
            // reset best cost bound and best path cost
            if (opt_)
            {
                bestCostBound_ = opt_->infiniteCost();
                bestPathCost_  = bestCostBound_;
            }
            // if no optimization objective set, set to infinity
            else
            {
                bestCostBound_ = base::Cost(std::numeric_limits<double>::infinity());
                bestPathCost_  = bestCostBound_;
            }
        }
        // function to free memory
        void AORRT::freeMemory()
        {
            // free all motions in the nearest neighbor structure
            if (nn_)
            {
                std::vector<Motion *> motions;
                nn_->list(motions);
                for (auto *m : motions)
                {
                    if (m->state_)
                        si_->freeState(m->state_);
                    if (m->control_)
                        siC_->freeControl(m->control_);
                    delete m;
                }
            }

            // also free the best solution path copy
            for (auto *s : bestPathStates_)
                if (s)
                    si_->freeState(s);
            bestPathStates_.clear();

            for (auto *c : bestPathControls_)
                if (c)
                    siC_->freeControl(c);
            bestPathControls_.clear();

            bestPathSteps_.clear();
        }

        // addRoot function is used to add a root node to the tree
        AORRT::Motion *AORRT::addRoot(const base::State *s)
        {
            // create a new motion
            auto *m = new Motion(siC_);
            si_->copyState(m->state_, s);
            siC_->nullControl(m->control_);
            // initialize motion parameters
            m->steps_   = 0;
            m->parent_  = nullptr;
            m->accCost_ = opt_->identityCost(); 
            nn_->add(m);
            return m;
        }

        // nearest neighbor to a given sampled state
        AORRT::Motion *AORRT::nearest(const base::State *s) const
        {
            // create a temporary motion for querying
            Motion query;
            query.state_ = const_cast<base::State *>(s);

            // perform nearest neighbor search
            std::vector<Motion *> result;
            result.reserve(1);

            // get nearest neighbor
            nn_->nearestK(&query, 1, result);

            // return nullptr if no result found
            if (result.empty())
                return nullptr;
            return result[0];
        }

        // function to grow the tree towards a target state
        AORRT::Motion *AORRT::growTreeTowards(const base::State *target)
        {
            // pick nearest node in tree
            Motion *nearestNode = nearest(target);
            if (!nearestNode)
                return nullptr;

            // sample a random control & duration
            if (!controlSampler_)
                controlSampler_ = siC_->allocControlSampler();

            Control *rctrl = siC_->allocControl();
            controlSampler_->sample(rctrl); // can give bias this toward target if needed

            unsigned int cd = rng_.uniformInt(
                siC_->getMinControlDuration(),
                siC_->getMaxControlDuration());

            // propagate from nearest node using sampled control & duration
            base::State *resultState = si_->allocState();
            unsigned int achieved = siC_->propagateWhileValid(
                nearestNode->state_, rctrl, cd, resultState);

            // check if propagation was valid for entire duration 
            if (achieved < cd)
            {
                // propagation hit an obstacle or went invalid
                si_->freeState(resultState);
                siC_->freeControl(rctrl);
                return nullptr;
            }

            // calculate incremental cost and total cost-to-come
            base::Cost incCostMotion  = opt_->motionCost(nearestNode->state_, resultState);
            base::Cost incCostControl = opt_->controlCost(rctrl, cd);
            base::Cost incCost        = opt_->combineCosts(incCostMotion, incCostControl);

            // total new cost to reach the new motion
            base::Cost newCost = opt_->combineCosts(nearestNode->accCost_, incCost);

            // pruning check is done to see if new motion can improve best solution
            //  If no solution yet: bestCostBound_ should be infinity
            if (!opt_->isCostBetterThan(newCost, bestCostBound_))
            {
                si_->freeState(resultState);
                siC_->freeControl(rctrl);
                return nullptr;
            }

            // If new motion passes pruning, create and add to tree
            auto *m = new Motion(siC_);
            si_->copyState(m->state_, resultState);
            siC_->copyControl(m->control_, rctrl);
            m->steps_   = cd;
            m->parent_  = nearestNode;
            m->accCost_ = newCost;

            nn_->add(m);

            si_->freeState(resultState);
            siC_->freeControl(rctrl);

            return m;
        }

        // main solve function
        base::PlannerStatus AORRT::solve(const base::PlannerTerminationCondition &ptc)
        {
            // call the validity checker
            checkValidity();

            // get the goal region
            base::Goal *goal = pdef_->getGoal().get();
            auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

            // Add all valid start states as roots
            while (const base::State *st = pis_.nextStart())
            {
                addRoot(st);
            }

            // check if there are any valid initial states
            if (!nn_ || nn_->size() == 0)
            {
                OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
                return base::PlannerStatus::INVALID_START;
            }

            // allocate samplers if not done already
            if (!sampler_)
                sampler_ = si_->allocStateSampler();
            if (!controlSampler_)
                controlSampler_ = siC_->allocControlSampler();

            // create a pointer to track best solution motion found
            AORRT::Motion *bestSolutionMotion = nullptr;
            double bestGoalDist = std::numeric_limits<double>::infinity();

            // get intermediate solution callback function if any are registered
            const base::ReportIntermediateSolutionFn intermediateSolutionCallback =
                pdef_->getIntermediateSolutionCallback();

            // print start of planning 
            OMPL_INFORM("%s: Starting AO-RRT with %u states in the tree",
                        getName().c_str(), nn_->size());

            // temporary state for sampling
            base::State *sampled = si_->allocState();

            // main planning loop
            while (ptc == false)
            {
                // Sample target state for expansion with goalBias_ we sample from goal region
                if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
                    goal_s->sampleGoal(sampled);
                else
                    sampler_->sampleUniform(sampled);

                // Try to extend toward sampled state
                Motion *newMotion = growTreeTowards(sampled);
                // if growth failed, continue to next iteration
                if (!newMotion)
                    continue; 

                // Check for goal satisfaction
                double dist = 0.0;
                bool reachedGoal = goal->isSatisfied(newMotion->state_, &dist);

                // if goal is reached, check if we can update best solution
                if (reachedGoal)
                {
                    // if goal-reaching motion is better than our best, update global best
                    if (bestSolutionMotion == nullptr ||
                        opt_->isCostBetterThan(newMotion->accCost_, bestPathCost_))
                    {
                        bestSolutionMotion = newMotion;
                        bestGoalDist       = dist;
                        bestPathCost_      = newMotion->accCost_;
                        bestCostBound_     = newMotion->accCost_; 

                        // Rebuild bestPathStates 
                        for (auto *s : bestPathStates_)
                            if (s)
                                si_->freeState(s);
                        bestPathStates_.clear();

                        for (auto *c : bestPathControls_)
                            if (c)
                                siC_->freeControl(c);
                        bestPathControls_.clear();
                        bestPathSteps_.clear();

                        // move backward through parents to collect the solution path
                        Motion *trav = bestSolutionMotion;
                        // loop to add each state/control/steps to best path
                        while (trav != nullptr)
                        {
                            bestPathStates_.push_back(si_->cloneState(trav->state_));
                            if (trav->parent_ != nullptr)
                            {
                                bestPathControls_.push_back(siC_->cloneControl(trav->control_));
                                bestPathSteps_.push_back(trav->steps_);
                            }
                            trav = trav->parent_;
                        }

                        // print solution found message
                        OMPL_INFORM("%s: Found solution with cost %.2f",
                                    getName().c_str(),
                                    bestPathCost_.value());

                        // check and call intermediate solution callback if registered
                        if (intermediateSolutionCallback)
                        {
                            // PlannerData wants start→goal, so we reverse
                            std::vector<const base::State *> solStatesFwd;
                            solStatesFwd.reserve(bestPathStates_.size());
                            for (auto it = bestPathStates_.rbegin();
                                 it != bestPathStates_.rend(); ++it)
                            {
                                solStatesFwd.push_back(*it);
                            }

                            intermediateSolutionCallback(this, solStatesFwd, bestPathCost_);
                        }
                    }
                }
                // if goal not reached, check if we can update closest-to-goal
                else
                {
                    if (bestSolutionMotion == nullptr && dist < bestGoalDist)
                    {
                        bestGoalDist = dist;
                    }
                }
            }

            // build final solution path if we have one
            bool solved = false;
            bool approximate = false;

            // check if we found an exact solution or an approximate one
            if (bestSolutionMotion == nullptr)
            {
                approximate = true;
            }
            else
            {
                solved = true;
            }

            // if we have a best solution motion, construct the PathControl
            if (bestSolutionMotion != nullptr)
            {
                // Construct a PathControl from start→goal using bestPath* we stored.
                auto path(std::make_shared<ompl::control::PathControl>(si_));

                // Reverse the bestPath* vectors to get start→goal order
                std::vector<ompl::base::State *> fwdStates;
                std::vector<ompl::control::Control *> fwdCtrls;
                std::vector<unsigned int> fwdSteps;
                for (int i = (int)bestPathStates_.size() - 1; i >= 0; --i)
                {
                    fwdStates.push_back(bestPathStates_[i]);
                }
                // Reverse-copy controls in the same order
                for (int i = (int)bestPathControls_.size() - 1; i >= 0; --i)
                {
                    fwdCtrls.push_back(bestPathControls_[i]);
                    fwdSteps.push_back(bestPathSteps_[i]);
                }

                // Append states and controls to path
                if (!fwdStates.empty())
                {
                    for (std::size_t i = 0; i + 1 < fwdStates.size(); ++i)
                    {
                        path->append(
                            fwdStates[i],
                            fwdCtrls[i],
                            fwdSteps[i] * siC_->getPropagationStepSize());
                    }
                    // final state without control
                    path->append(fwdStates.back());
                }

                pdef_->addSolutionPath(path,
                                       approximate /* isApprox */,
                                       bestGoalDist,
                                       getName());
            }

            OMPL_INFORM("%s: Tree size: %u states", getName().c_str(),
                        nn_ ? nn_->size() : 0);

            return {solved, approximate};
        }

        // function to get planner data
        void AORRT::getPlannerData(ompl::base::PlannerData &data) const
        {
            Planner::getPlannerData(data);

            // if no nearest neighbor structure, return
            if (!nn_)
                return;

            // get all motions from nearest neighbor structure
            std::vector<Motion *> motions;
            nn_->list(motions);

            // add all motions to planner data
            for (auto *m : motions)
            {
                if (m->parent_ == nullptr)
                {
                    // Root nodes are start vertices
                    data.addStartVertex(
                        ompl::base::PlannerDataVertex(m->state_));
                }
                else
                {
                    // Add an edge parent -> child 
                    data.addEdge(
                        ompl::base::PlannerDataVertex(m->parent_->state_),
                        ompl::base::PlannerDataVertex(m->state_));
                }
            } 
            if (!bestPathStates_.empty())
            {
                // add best path to goal vertex
                data.addGoalVertex(
                    ompl::base::PlannerDataVertex(bestPathStates_.front()));
            }
        }

    } // namespace control
} // namespace ompl
