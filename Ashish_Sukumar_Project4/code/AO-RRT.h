///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Ashish Sukumar
//////////////////////////////////////
#ifndef AORRT_H
#define AORRT_H

#include "ompl/control/SpaceInformation.h"
#include "ompl/base/Planner.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/StateSampler.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/Cost.h"
#include "ompl/util/RandomNumbers.h"

#include <vector>
#include <memory>
#include <limits>

namespace ompl
{
    namespace control
    {
        // TODO: Implement AO-RRT as described

        // class AORRT : public base::Planner
        // {
        // };

        // define class for AO-RRT
        class AORRT : public base::Planner
        {
        public:
            // create constructor
            AORRT(const SpaceInformationPtr &si);

            // create destructor for cleaning up memory
            ~AORRT() override;

            // override setup function 
            void setup() override;

            // override clear function
            void clear() override;

            // create a solve override function
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            // override getPlannerData function
            void getPlannerData(base::PlannerData &data) const override;

            // Create function to set and get goal bias
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            double getGoalBias() const
            {
                return goalBias_;
            }

            // create a function to set the nearest neighbor structure
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }
        protected:
            // create a node named Motion
            class Motion
            {
            public:
            // default constructor
                Motion() = default;
            // constructor with the state information pointer
                Motion(const SpaceInformation *si)
                  : state_(si->allocState()),
                    control_(si->allocControl())
                {
                }
                // destructor to free state and control
                ~Motion() = default;

                base::State    *state_{nullptr};   // state of this node
                Control        *control_{nullptr}; 
                unsigned int    steps_{0};         // how many integration steps that control was applied
                Motion         *parent_{nullptr};  
                base::Cost      accCost_{0};       // accumulated cost
            };

            // functions to grow the tree towards a target state
            Motion *addRoot(const base::State *s);

            // nearest neighbor to a given sampled state
            Motion *nearest(const base::State *s) const;

            // Try to extend tree from nearest node toward target sample
            Motion *growTreeTowards(const base::State *target);

            // function to free memory
            void freeMemory();

            // distance function between two motions
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state_, b->state_);
            }

            // these pointers are used for sampling 
            base::StateSamplerPtr sampler_;
            ControlSamplerPtr     controlSampler_;

            // Space information pointers
            const SpaceInformation *siC_{nullptr};

            // nearest neighbor structure
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            // Random number generator 
            RNG rng_;

            // optimization objective pointer used for cost calculations
            base::OptimizationObjectivePtr opt_;

            // best solution found so far
            base::Cost bestCostBound_;

            // the best state/control/steps sequences found so far
            std::vector<base::State *> bestPathStates_;
            std::vector<Control *>     bestPathControls_;
            std::vector<unsigned int>  bestPathSteps_;
            base::Cost                 bestPathCost_;
            double goalBias_{0.05};  // probability of sampling directly from goal region
        };
    }  // namespace control 
}  // namespace ompl

#endif