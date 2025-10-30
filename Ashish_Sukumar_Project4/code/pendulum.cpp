///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Ashish Sukumar
//////////////////////////////////////

#include <ompl/base/spaces/SO2StateSpace.h>
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
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

#include <ompl/tools/benchmark/Benchmark.h>

#include <memory>
#include <cmath>
#include <iostream>
#include <fstream>  
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace ot = ompl::tools;

// Projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{

public:
    explicit PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space) {}

    unsigned int getDimension() const override { 
        // TODO: The dimension of your projection for the pendulum
        // pendulum state is [theta, omega] 
        return 2; 
    }

    void project(const ompl::base::State *state,
                 Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
        const auto *compound = state->as<ompl::base::CompoundStateSpace::StateType>();
        // extract values of theta and omega
        const auto *thetast  = compound->as<ompl::base::SO2StateSpace::StateType>(0);
        const auto *omegast  = compound->as<ompl::base::RealVectorStateSpace::StateType>(1);
        // set the values of the projection
        projection[0] = thetast->value;
        projection[1] = omegast->values[0];
    }
};

// q = [theta, omega], u = [tau]
void pendulumODE(const oc::ODESolver::StateType &q,
                 const oc::Control *control,
                 oc::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    // angle and angular velocity
    const double theta = q[0];
    const double omega = q[1];

    // control is the torque applied
    const double tau = control
        ->as<oc::RealVectorControlSpace::ControlType>()
        ->values[0];

    const double g = 9.81; // constant value

    // compute the derivative
    qdot.resize(2); // pendulum has 2 state variables
    qdot[0] = omega;
    qdot[1] = -g * std::cos(theta) + tau;
}

// Creating a SimpleSetup for the pendulum 
oc::SimpleSetupPtr createPendulum(double torque)
{
    using namespace ompl;

    // State space = SO2 (theta) X R (omega)
    auto thetaSpace = std::make_shared<base::SO2StateSpace>();     
    auto omegaSpace = std::make_shared<base::RealVectorStateSpace>(1); 
    {
        base::RealVectorBounds wBounds(1);
        // omega bounds are set
        wBounds.setLow(-10.0);
        wBounds.setHigh(10.0);
        omegaSpace->setBounds(wBounds);
    }
    // compound state space is given by the combination of theta and omega spaces
    base::StateSpacePtr stateSpace = thetaSpace + omegaSpace;

    // register the projection so that KPIECE1 can use it 
    stateSpace->registerDefaultProjection(std::make_shared<PendulumProjection>(stateSpace.get()));

    // Control space is given by the torque applied
    auto controlSpace = std::make_shared<control::RealVectorControlSpace>(stateSpace, 1);
    {
        // set bounds for the control (torque)
        base::RealVectorBounds cb(1);
        // torque bounds are set
        cb.setLow(-torque);
        cb.setHigh(torque);
        controlSpace->setBounds(cb);
    }

    auto ss = std::make_shared<control::SimpleSetup>(controlSpace);

    // Validity checker is used to check if any values are out of bounds
    ss->setStateValidityChecker(
        [&ss](const base::State *st)
        {
            // compond
            const auto *compound = st->as<base::CompoundStateSpace::StateType>();
            const auto *omegaPart = compound->as<base::RealVectorStateSpace::StateType>(1);
            const double w = omegaPart->values[0];
            return (w >= -10.0 && w <= 10.0);
        });

    // State propagator using the ODE 
    auto si = ss->getSpaceInformation();
    // creating the ODE solver
    auto odeSolver = std::make_shared<control::ODEBasicSolver<> >(si, &pendulumODE);
    si->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver));

    // set the propagation step size used for integration to
    si->setPropagationStepSize(0.01);
    // set min and max control which is used to sample control 
    si->setMinMaxControlDuration(1, 20);

    // Define start and goal states
    // Start state
    ob::ScopedState<> start(stateSpace);
    start[0] = -M_PI / 2.0;  // theta
    start[1] = 0.0;          // omega

    // Goal state
    ob::ScopedState<> goal(stateSpace);
    goal[0] =  M_PI / 2.0;   // theta
    goal[1] =  0.0;          // omega

    // set the start and goal states with a threshold
    ss->setStartAndGoalStates(start, goal, 0.1);
    ss->setup();
    return ss;
}

// Planning algorithms for the pendulum
void planPendulum(oc::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
    using namespace ompl;

    // Planner selection
    ob::PlannerPtr planner;
    // get the space information
    auto si = ss->getSpaceInformation();
    // select the planner based on user choice
    switch (choice)
    {
        // 1 is RRT
        case 1: planner = std::make_shared<oc::RRT>(si); break;
        // 2 is EST
        case 2: planner = std::make_shared<oc::EST>(si); break;
        // any number other than 1 and 2 will be taken as KPIECE1
        default:
        {
            auto kp = std::make_shared<oc::KPIECE1>(si);
            planner = kp;
            break;
        }
    }
    // set the planner
    ss->setPlanner(planner);

    // set the solve time
    const double solveTimeSeconds = 20.0;
    // attempt to solve the planning problem
    if (ss->solve(solveTimeSeconds))
    {
        // get the solution path
        auto path = ss->getSolutionPath();
        // print the success message if solved
        std::cout << "Solved with planner choice " << choice << "!\n";
        std::cout << "Path has " << path.getControlCount() << " control segments.\n";

       // export the path to pendulum.txt
        {
            // *pc is a pointer to the type of path
            oc::PathControl *pc = path.as<oc::PathControl>();
            // siC is the space information for control
            auto siC             = ss->getSpaceInformation();
            // dt is the propagation step size
            const double dt      = siC->getPropagationStepSize();

            std::ofstream out("pendulum.txt");
            double T = 0.0;

            // initial state with no control is written 
            {
                const ob::State *s0 = pc->getState(0);
                 const auto *compound = s0->as<ob::CompoundStateSpace::StateType>();
                const auto *thetaSt  = compound->as<ob::SO2StateSpace::StateType>(0);
                const auto *omegaSt  = compound->as<ob::RealVectorStateSpace::StateType>(1);
                out << T << " " << thetaSt->value << " " << omegaSt->values[0] << " " << 0.0 << "\n";
            }
            
            // loop to write each control segment
            for (std::size_t seg = 0; seg < pc->getControlCount(); ++seg)
            {
                const oc::Control *u = pc->getControl(seg);
                const double tau = u
                    ->as<oc::RealVectorControlSpace::ControlType>()
                    ->values[0];

                const double dur = pc->getControlDurations()[seg];
                unsigned int steps = std::max(1u, static_cast<unsigned int>(std::ceil(dur / dt)));

                ob::State *cur  = siC->cloneState(pc->getState(seg));
                ob::State *next = siC->allocState();

                for (unsigned int k = 0; k < steps; ++k)
                {
                    // one internal step of the integrator
                    siC->propagateWhileValid(cur, u, 1, next);
                    T += dt;

                    const auto *compound = next->as<ob::CompoundStateSpace::StateType>();
                    const auto *thetaSt  = compound->as<ob::SO2StateSpace::StateType>(0);
                    const auto *omegaSt  = compound->as<ob::RealVectorStateSpace::StateType>(1);

                    out << T << " " << thetaSt->value << " " << omegaSt->values[0] << " " << tau << "\n";
                    siC->copyState(cur, next);
                }

                siC->freeState(next);
                siC->freeState(cur);
            }

            // check and write the final state
            if (pc->getStateCount() > 0)
            {
                const ob::State *sf = pc->getState(pc->getStateCount() - 1);
                const auto *compound = sf->as<ob::CompoundStateSpace::StateType>();
                const auto *thetaSt  = compound->as<ob::SO2StateSpace::StateType>(0);
                const auto *omegaSt  = compound->as<ob::RealVectorStateSpace::StateType>(1);
                out << T << " " << thetaSt->value << " " << omegaSt->values[0] << " " << 0.0 << "\n";
            }

            out.close();
            std::cout << "[OK] Saved dense path to pendulum.txt\n";
        }
    }
    // if no solution is found 
    else
    {
        std::cout << "No solution found with planner choice " << choice << " within time limit.\n";
    }
}

// Benchmarking the pendulum
void benchmarkPendulum(oc::SimpleSetupPtr &ss)
{
    // TODO: Do some benchmarking for the pendulum
    using namespace ompl;

    // get the space information
    auto si = ss->getSpaceInformation();
    ot::Benchmark benchmark(*ss, "pendulum_benchmark");

    // add planners to benchmark
    benchmark.addPlanner(std::make_shared<oc::RRT>(si));
    benchmark.addPlanner(std::make_shared<oc::EST>(si));
    benchmark.addPlanner(std::make_shared<oc::KPIECE1>(si));

    // set benchmark request parameters
    ot::Benchmark::Request req;
    // set max time, max memory, number of runs and display progress
    req.maxTime = 20.0;
    req.maxMem  = 1000.0;
    req.runCount = 20;
    req.displayProgress = true;

    // run the benchmark
    benchmark.benchmark(req);
    // save the benchmark results to a log file
    benchmark.saveResultsToFile("pendulum_benchmark.log");
}

int main(int argc, char **argv)
{
    int choice;
    do {
        std::cout << "Plan or Benchmark?\n"
                  << " (1) Plan\n"
                  << " (2) Benchmark\n";
        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do {
        std::cout << "Torque?\n"
                  << " (1)  3\n"
                  << " (2)  5\n"
                  << " (3) 10\n";
        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    oc::SimpleSetupPtr ss = createPendulum(torque);

    if (choice == 1)
    {
        int planner;
        do {
            std::cout << "What Planner?\n"
                      << " (1) RRT\n"
                      << " (2) EST\n"
                      << " (3) KPIECE1\n";
            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    else if (choice == 2)
    {
        benchmarkPendulum(ss);
    }
    else
    {
        std::cerr << "How did you get here? Invalid choice.\n";
    }

    return 0;
}
