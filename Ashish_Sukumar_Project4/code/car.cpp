///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Ashish Sukumar
//////////////////////////////////////

#include <iostream>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerStatus.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/PlannerData.h>

#include "CollisionChecking.h"
#include "SST.h"
#include "AO-RRT.h"

#include <memory>
#include <cmath>
#include <vector>
#include <fstream>  

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace ot = ompl::tools;

// Projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space) {}

    unsigned int getDimension() const override { 
        // TODO: The dimension of your projection for the car
        return 2; 
    }

    void project(const ompl::base::State *state,
                 Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // get the state type
        const auto *rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
        // set the projection values (x, y)
        projection[0] = rv->values[0]; // x
        projection[1] = rv->values[1]; // y
    }
};

// ODE for the car's dynamics
void carODE(const ompl::control::ODESolver::StateType & q,
            const ompl::control::Control * control,
            ompl::control::ODESolver::StateType & qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
    // q = [x, y, theta, v]
    const double theta = q[2];
    const double v     = q[3];

    // control = [a, w]
    const double a =
        control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0];
    const double w =
        control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1];

    qdot.resize(4);
    // 
    qdot[0] = v * std::cos(theta); // x dot
    qdot[1] = v * std::sin(theta); // y dot
    qdot[2] = w;                   // theta dot
    qdot[3] = a;                   // v dot
}

void makeStreet(std::vector<Rectangle> &  obstacles )
{
    // Do not change the obstacles here. These are the same obstacles used for grading.
    obstacles.emplace_back(Rectangle{5.0, -2.0, 7, 5});
    obstacles.emplace_back(Rectangle{-4, 5, 16, 2});
    obstacles.emplace_back(Rectangle{-4, -2, 7, 4});
    obstacles.emplace_back(Rectangle{8, 3, 4, 2});
}

// Creating a SimpleSetup for the car
ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles )
{
    // State space: [x, y, theta, v]
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(4);

    // bounds for x, y, theta, v
    ompl::base::RealVectorBounds sb(4);
    sb.setLow(0, -6.0);   sb.setHigh(0, 15.0);  // x
    sb.setLow(1, -4.0);   sb.setHigh(1, 10.0);  // y
    sb.setLow(2, -M_PI);  sb.setHigh(2,  M_PI); // theta
    sb.setLow(3, -3.0);   sb.setHigh(3,  3.0);  // v
    space->setBounds(sb);

    // register the projection
    space->registerDefaultProjection(std::make_shared<CarProjection>(space.get()));

    // Control space: [a, w]
    auto controlSpace = std::make_shared<ompl::control::RealVectorControlSpace>(space, 2);
    // bounds for a, w
    ompl::base::RealVectorBounds cb(2);
    cb.setLow(0, -2.0); cb.setHigh(0,  2.0);  // accel
    cb.setLow(1, -1.5); cb.setHigh(1,  1.5);  // yaw rate
    controlSpace->setBounds(cb);

    // create the SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(controlSpace);

    // Validity checker for the car
    ss->setStateValidityChecker(
        [&obstacles, space](const ompl::base::State *st) -> bool
        {
            // reject anything outside the bounds
            if (!space->satisfiesBounds(st)) return false;

            // get the state values
            const auto *rv = st->as<ompl::base::RealVectorStateSpace::StateType>();
            double x     = rv->values[0];
            double y     = rv->values[1];
            double theta = rv->values[2];
            double v     = rv->values[3];

            // reject anything with speed outside [-3, 3]
            if (v < -3.0 || v > 3.0) return false;

            constexpr double sideLength = 1.0; // 1x1 square car
            // check collision using isValidSquare
            return isValidSquare(x, y, theta, sideLength, obstacles);
        }
    );
    
    auto si = ss->getSpaceInformation();

    // ODE solver for the car
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<> >(si, &carODE);
    // set the state propagator
    si->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    // set other space information parameters
    si->setPropagationStepSize(0.02);
    si->setMinMaxControlDuration(5, 50);
    si->setStateValidityCheckingResolution(0.005); // more frequent checks along edges

    // Start and goal
    ompl::base::ScopedState<> start(space), goal(space);
    start[0] = -6.0;  start[1] = -4.0;  start[2] = 0.0; start[3] = 0.0;
    goal [0] =  7.0;  goal [1] =  4.0;  goal [2] = 0.0; goal [3] = 0.0;

    // Goal tolerance
    ss->setStartAndGoalStates(start, goal, 0.25);

    ss->setup();
    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    using namespace ompl;

    auto si = ss->getSpaceInformation();

    ob::PlannerPtr planner;
    switch (choice)
    {
        // choose the planner based on user choice
        case 1: planner = std::make_shared<oc::KPIECE1>(si); break;
        // 2 is SST
        case 2: planner = std::make_shared<oc::SST>(si);     break;
        default:
        {
            auto aorrt = std::make_shared<oc::AORRT>(si);
            aorrt->setGoalBias(0.05);
            planner = aorrt;
            break;
        }
    }

    ss->setPlanner(planner);

    // solve
    const double solveTimeSeconds = 1000.0; 
    // attempt to solve the planning problem
    if (ss->solve(solveTimeSeconds))
    {
        // if solution is found, print it
        auto pathAny = ss->getSolutionPath();
        auto *pc = pathAny.as<oc::PathControl>();

        std::cout << "Solved with planner choice " << choice << "!\n";
        std::cout << "Path has " << pc->getControlCount() << " control segments.\n";

        // print to path.txt
        auto siC = ss->getSpaceInformation();
        const double dt = siC->getPropagationStepSize();

        std::ofstream out("path.txt");
        double T = 0.0; 

        // initial state without control
        {
            // get the initial state
            const ob::State *s0 = pc->getState(0);
            // get the state values
            const auto *rv0 = s0->as<ob::RealVectorStateSpace::StateType>();
            out << T << " " << rv0->values[0] << " " << rv0->values[1] << " "
                << rv0->values[2] << " " << rv0->values[3] << " "
                << 0.0 << " " << 0.0 << "\n";
        }

        // iterate through each control segment
        for (std::size_t seg = 0; seg < pc->getControlCount(); ++seg)
        {
            const oc::Control *u = pc->getControl(seg);
            const double *uv = u->as<oc::RealVectorControlSpace::ControlType>()->values;
            const double a = uv[0], w = uv[1];

            const double dur = pc->getControlDurations()[seg];
            unsigned int steps = std::max(1u, static_cast<unsigned int>(std::ceil(dur / dt)));

            ob::State *cur  = siC->cloneState(pc->getState(seg));
            ob::State *next = siC->allocState();

            for (unsigned int k = 0; k < steps; ++k)
            {
                siC->propagateWhileValid(cur, u, 1, next);
                T += dt;

                // record clamped state values (remain inside bounds)
                const auto *rv = next->as<ob::RealVectorStateSpace::StateType>();
                out << T << " " << rv->values[0] << " " << rv->values[1] << " "
                    << rv->values[2] << " " << rv->values[3] << " "
                    << a << " " << w << "\n";

                siC->copyState(cur, next);
            }

            siC->freeState(next);
            siC->freeState(cur);
        }

        {
            const ob::State *sf = pc->getState(pc->getStateCount() - 1);
            const auto *rvf = sf->as<ob::RealVectorStateSpace::StateType>();
            out << T << " " << rvf->values[0] << " " << rvf->values[1] << " "
                << rvf->values[2] << " " << rvf->values[3] << " "
                << 0.0 << " " << 0.0 << "\n";
        }

        std::cout << "Saved dense path to path.txt\n";
    }
    // if no solution is found print it
    else
    {
        std::cout << "No solution found with planner choice "
                  << choice << " within time limit.\n";
    }
}

// Benchmarking the car
void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the car
    using namespace ompl;

    // get the space information
    oc::SpaceInformationPtr si = ss->getSpaceInformation();

    // create the benchmark object
    ot::Benchmark benchmark(*ss, "car_benchmark");

    // add planners to benchmark
    benchmark.addPlanner(std::make_shared<oc::KPIECE1>(si));
    benchmark.addPlanner(std::make_shared<oc::SST>(si));
    {
        auto aorrt = std::make_shared<oc::AORRT>(si);
        aorrt->setGoalBias(0.05);
        benchmark.addPlanner(aorrt);
    }

    // set benchmark request parameters
    ot::Benchmark::Request req;
    req.maxTime = 200.0;
    req.maxMem = 1000.0;
    req.runCount = 20;
    req.displayProgress = true;

    benchmark.benchmark(req);
    benchmark.saveResultsToFile("car_benchmark.log");
}

int main(int argc, char **argv)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? \n (1) Plan\n (2) Benchmark\n";
        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? \n (1) KPIECE1\n (2) SST\n (3) AO-RRT\n";
            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    else if (choice == 2)
        benchmarkCar(ss);
    else
        std::cerr << "How did you get here? Invalid choice.\n";

    return 0;
}
