/** @file Simulator.hpp
 *
 * Contains the Simulator class, which runs simulations to test the performance of ABT.
 */
#ifndef SOLVER_SIMULATOR_HPP_
#define SOLVER_SIMULATOR_HPP_

#include "global.hpp"

#include "solver/Agent.hpp"

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/ModelChange.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/Options.hpp"
#include "solver/abstract-problem/State.hpp"

namespace solver {
class HistorySequence;
class Model;
class Solver;

/** A class for running simulations to test the performance of ABT.
 *
 * This class stores the actual history of the simulation in a HistorySequence (one that is
 * not associated with the solver), and also keeps track of the amount of time taken to do
 * many of the individual tasks done by the solver.
 */
class Simulator {
public:
    /** Constructs a new simulator. The given model will be owned by the simulator and will be the
     * model used for the actual simulation, whereas the solver will have its own, distinct model.
     *
     * Using two different model objects allows this code to be tested in situations where the
     * Solver's model can be a flawed representation of the actual problem, which is commonly the
     * case in real-world problems.
     *
     * The flag "hasDynamicChanges" selects between two types of changes for this simulation;
     * "dynamic changes" are assumed to represent an update to reflect a changing environment,
     * which means that when the model is updated only the subtree of the belief tree that starts
     * at the current node will be affected.
     *
     * On the other hand, "static changes" represent an update to reflect a more accurate
     * model of the underlying reality. This model is assumed to apply backwards in time, and hence
     * belief states that are in the "past", as well as the current belief of the solver, will
     * be updated based on the new changes.
     */
    Simulator(std::unique_ptr<Model> model, Solver *solver, bool hasDynamicChanges);
    ~Simulator() = default;
    _NO_COPY_OR_MOVE(Simulator);

    /** Returns the model used by the simulator. */
    Model *getModel() const;
    /** Returns the agent that is embedded in the simulator. */
    Agent *getAgent() const;
    /** Returns the solver being tested by this simulator. */
    Solver *getSolver() const;
    /** Returns the model being used by the solver. */
    Model *getSolverModel() const;

    /** Returns the actual current state of the simulation. */
    State const *getCurrentState() const;
    /** Returns the history of the simulation. */
    HistorySequence *getHistory() const;
    /** Returns the number of steps taken in this simulation. */
    long getStepCount() const;

    /** Returns the total time spent on changes to the solver's model and policy. */
    double getTotalChangingTime() const;
    /** Returns the total time spent replenishing particles. */
    double getTotalReplenishingTime() const;
    /** Returns the total time spent on straight improvements to the policy (i.e. generating new
     * histories.
     */
    double getTotalImprovementTime() const;
    /** Returns the total time spent on pruning the tree. */
    double getTotalPruningTime() const;

    /** Sets a sequence of changes to be used for this simulation. */
    void setChangeSequence(ChangeSequence sequence);
    /** Loads a sequence of changes from a file at the given path. */
    void loadChangeSequence(std::string path);

    /** Sets the maximum step count for this simulator to the given number. */
    void setMaxStepCount(long maxStepCount);

    /** Runs a full simulation, returning the total discounted reward. */
    double runSimulation();
    /** Steps the simulation forward one step.
     *
     * a false return value means the simulation has ended.
     */
    bool stepSimulation();
    /** Handles the given vector of changes. The "areDynamic" flag determines whether the update
     * will be dynamic (only affecting the current subtree), or static (affecting the entire tree,
     * including past states).
     *
     * Returns false if the changes were invalid or failed in some way.
     */
    bool handleChanges(std::vector<std::unique_ptr<ModelChange>> const &changes,
            bool areDynamic = true);

private:
    /** The simulator's model, which is used to generate the actual simulation history. */
    std::unique_ptr<Model> model_;
    /** The solver being tested. */
    Solver *solver_;
    /** The configuration settings for the solver / solver's model. */
    Options const *options_;
    /** The model owned by the solver. */
    Model *solverModel_;

    /** The agent that will be queried for actions. */
    std::unique_ptr<Agent> agent_;

    /** True iff dynamic changes will be used. */
    bool hasDynamicChanges_;
    /** A pre-set sequence of changes for this simulation. */
    ChangeSequence changeSequence_;

    /** The number of simulation steps taken so far. */
    long stepCount_;
    /** The maximum number of simulation steps to take. */
    long maxStepCount_;
    /** The current cumulative discount. */
    double currentDiscount_;
    /** The total discounted reward. */
    double totalDiscountedReward_;
    /** The actual history of the simulation. */
    std::unique_ptr<HistorySequence> actualHistory_;

    /** The total time spent on changes to the solver's model and policy. */
    double totalChangingTime_;
    /** The total time spent on replenishing particles. */
    double totalReplenishingTime_;
    /** The total time spent on improving the policy, via new histories. */
    double totalImprovementTime_;
    /** The total time spent on pruning the tree. */
    double totalPruningTime_;
};
} /* namespace solver */

#endif /* SOLVER_SIMULATOR_HPP_ */

