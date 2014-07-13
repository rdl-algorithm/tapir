/** @file RockSampleModel.hpp
 *
 * Contains RockSampleModel, which implements the core Model interface for the RockSample POMDP.
 */
#ifndef ROCKSAMPLE_MODEL_HPP_
#define ROCKSAMPLE_MODEL_HPP_

#include <ios>                          // for ostream
#include <memory>                       // for unique_ptr
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"       // for State
#include "solver/abstract-problem/ModelChange.hpp"

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/enumerated_observations.hpp"

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model

#include "position_history.hpp"
#include "smart_history.hpp"
#include "RockSampleMdpSolver.hpp"
#include "RockSampleOptions.hpp"
#include "RockSampleAction.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace solver {
class ActionMapping;
class StatePool;
class DiscretizedPoint;
} /* namespace solver */

/** A namespace to hold the various classes used for the RockSample POMDP model. */
namespace rocksample {
class RockSampleMdpSolver;
class RockSampleState;
class RockSampleObservation;

/** Represents a change in the RockSample model. */
struct RockSampleChange : public solver::ModelChange {
    /** The change type for this change - this should be one of:
     * - "Add Obstacles" - to add new obstacles to the RockSample problem.
     * - "Remove Obstacles" - to remove existing obstacles from the RockSample problem.
     */
    std::string changeType = "";
    /** The first row number where the change applies. */
    long i0 = 0;
    /** The last row number where the change applies. */
    long i1 = 0;
    /** The first column number where the change applies. */
    long j0 = 0;
    /** The last column number where the change applies. */
    long j1 = 0;
};

/** The implementation of the Model interface for the RockSample POMDP.
 *
 * See this paper http://arxiv.org/ftp/arxiv/papers/1207/1207.4166.pdf
 * for a description of the RockSample problem.
 *
 * This class inherits from shared::ModelWithProgramOptions in order to use custom text-parsing
 * functionality to select many of the core ABT parameters, allowing the configuration options
 * to be changed easily via the configuration interface without having to recompile the code.
 */
class RockSampleModel : public shared::ModelWithProgramOptions {
friend class RockSampleMdpSolver;
  public:
    /** Constructs a new RockSampleModel instance with the given random number engine, and the
     * given set of configuration options.
     */
    RockSampleModel(RandomGenerator *randGen, std::unique_ptr<RockSampleOptions> options);
    ~RockSampleModel() = default;
    _NO_COPY_OR_MOVE(RockSampleModel);

    /** The different levels of history-based heuristic options that can be used. */
    enum RSActionCategory : int {
        /** The lowest level - all actions will be treated the same way. */
        ALL = 0,
        /** Legal actions get special treatment. */
        LEGAL = 1,
        /** Some types of actions are "preferred". */
        PREFERRED = 2
    };

    /** Parses an action category (all/legal/preferred) from a string. */
    RSActionCategory parseCategory(std::string categoryString) {
        if (categoryString == "legal") {
            return RSActionCategory::LEGAL;
        } else if (categoryString == "preferred") {
            return RSActionCategory::PREFERRED;
        } else {
            return RSActionCategory::ALL;
        }
    }

    /**
     * Rocks are enumerated 0, 1, 2, ... ;
     * other cell types should be negative.
     */
    enum RSCellType : int {
        ROCK = 0,
        EMPTY = -1,
        GOAL = -2,
        OBSTACLE = -3,
    };


    /* ----------------------- Basic getters  ------------------- */
    /** Returns the number of rocks used in this model. */
    long getNumberOfRocks() {
        return nRocks_;
    }
    /** Returns the category of actions to cover in searches. */
    RSActionCategory getSearchActionCategory() {
        return searchCategory_;
    }
    /** Returns true if nodes should be initialized with preferred values. */
    bool usingPreferredInit() {
        return usingPreferredInit_;
    }
    /** Returns the initial q-value for preferred actions. */
    double getPreferredQValue() {
        return preferredQValue_;
    }
    /** Returns the initial visit count for preferred actions. */
    long getPreferredVisitCount() {
        return preferredVisitCount_;
    }

    /** Sets up the MDP solver for this model. */
    void makeMdpSolver() {
        mdpSolver_ = std::make_unique<RockSampleMdpSolver>(this);
        mdpSolver_->solve();
    }
    /** Returns the MDP solver for this model. */
    RockSampleMdpSolver *getMdpSolver() {
        return mdpSolver_.get();
    }

    /** Returns the starting position for this problem. */
    GridPosition getStartPosition() {
        return startPos_;
    }
    /** Returns the distance from the given grid position to the given rock (or -1 for the goal).
     *
     * A return value of -1 means the given rock / goal cannot be reached.
     */
    int getDistance(GridPosition p, int rockNo) {
        std::vector<std::vector<int>> *grid = nullptr;
        if (rockNo == -1) {
            grid = &goalDistances_;
        } else {
            grid = &rockDistances_[rockNo];
        }
        return (*grid)[p.i][p.j];
    }
    /** Returns true iff the given position is within the RockSample bounds. */
    bool isWithinBounds(GridPosition p) {
        return (p.i >= 0 && p.i < nRows_ && p.j >= 0 && p.j < nCols_);
    }
    /** Returns the cell type for the given position. */
    RSCellType getCellType(GridPosition p) {
        return envMap_[p.i][p.j];
    }
    /** Returns the grid position for the given rock. */
    GridPosition getRockPosition(int rockNo) {
        return rockPositions_[rockNo];
    }
    /** Calculates the probability that the sensor will be accurate, at the
     * given distance. */
    double getSensorCorrectnessProbability(double distance) {
        return (1 + std::pow(2, -distance / halfEfficiencyDistance_)) * 0.5;
    }


    /* --------------- The model interface proper ----------------- */
    virtual std::unique_ptr<solver::State> sampleAnInitState() override;
    /** Generates a state uniformly at random. */
    virtual std::unique_ptr<solver::State> sampleStateUninformed() override;
    virtual bool isTerminal(solver::State const &state) override;
    virtual bool isValid(solver::State const &state) override;


    /* -------------------- Black box dynamics ---------------------- */
    virtual std::unique_ptr<solver::State> generateNextState(
            solver::State const &state,
            solver::Action const &action,
            solver::TransitionParameters const */*tp*/) override;
    virtual std::unique_ptr<solver::Observation> generateObservation(
            solver::State const */*state*/,
            solver::Action const &action,
            solver::TransitionParameters const */*tp*/,
            solver::State const &nextState) override;
    virtual double generateReward(
            solver::State const &state,
            solver::Action const &action,
            solver::TransitionParameters const */*tp*/,
            solver::State const */*nextState*/) override;
    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action) override;


    /* -------------- Methods for handling model changes ---------------- */
    virtual void applyChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
                 solver::Solver *solver) override;


    /* ------------ Methods for handling particle depletion -------------- */
    /** Generates particles for RockSample using a particle filter from the previous belief.
      *
      * For each previous particle, possible next states are calculated based on consistency with
      * the given action and observation. These next states are then added to the output vector
      * in accordance with their probability of having been generated.
      */
    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action, solver::Observation const &obs,
            long nParticles,
            std::vector<solver::State const *> const &previousParticles) override;

    /** Generates particles for RockSample according to an uninformed prior.
     *
     * This method uses the fully observed part of the state, and samples the previous rock
     * states uniformly at random. For each sample, a single step is generated with the action,
     * and only states consistent with the observation are kept.
     *
     * NOTE: If this method gets called, things are going badly wrong :(
     */
    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs,
            long nParticles) override;


    /* ------------------- Pretty printing methods --------------------- */
    /** Displays an individual cell of the map. */
    virtual void dispCell(RSCellType cellType, std::ostream &os);
    virtual void drawEnv(std::ostream &os) override;
    /** Draws a single grid of distances to a specific rock. */
    virtual void drawDistances(std::vector<std::vector<int>> &grid, std::ostream &os);
    virtual void drawSimulationState(solver::BeliefNode const *belief,
            solver::State const &state,
            std::ostream &os) override;


    /* ---------------------- Basic customizations  ---------------------- */
    virtual double getDefaultHeuristicValue(solver::HistoryEntry const *entry,
            solver::State const *state, solver::HistoricalData const *data) override;

    virtual std::unique_ptr<solver::Action> getRolloutAction(solver::HistoryEntry const *entry,
            solver::State const *state, solver::HistoricalData const *data) override;


    /* ------- Customization of more complex solver functionality  --------- */
    /** Returns all of the available actions in the RockSample POMDP, in enumerated order. */
    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>> getAllActionsInOrder();
    virtual std::unique_ptr<solver::HistoricalData> createRootHistoricalData() override;
    virtual std::unique_ptr<solver::ActionPool> createActionPool(solver::Solver *solver) override;

    /** Returns all of the possible observations in the RockSample POMDP, in enumerated order. */
    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>> getAllObservationsInOrder();
    virtual std::unique_ptr<solver::ObservationPool> createObservationPool(solver::Solver *solver) override;

    virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;


    /* ----------- Non-virtual methods for RockSampleModel ------------- */
    /** Returns true iff the given position is a valid grid square for the robot to be in. */
    bool isValid(GridPosition position);

    /** Generates an adjacent position without doing bounds checks or legality checks. */
    GridPosition makeAdjacentPosition(GridPosition position, ActionType actionType);
    /** Generates the next position for the given position and action. */
    std::pair<GridPosition, bool> makeNextPosition(GridPosition pos, ActionType actionType);
    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    std::pair<std::unique_ptr<RockSampleState>, bool> makeNextState(
            RockSampleState const &state,
            RockSampleAction const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    std::unique_ptr<RockSampleObservation> makeObservation(
            RockSampleAction const &action,
            RockSampleState const &nextState);
    /** Retrieves the reward via the next state. */
    double makeReward(RockSampleState const &state,
            RockSampleAction const &action,
            RockSampleState const &nextState,
            bool isLegal);

  private:
    /**
     * Finds and counts the rocks on the map, and initializes the required
     * data structures and variables.
     */
    void initialize();

    /** For each cell, calculates the distance to the nearest goal and the */
    void recalculateAllDistances();

    /** For each cell, calculates the distance to the nearest target. If no target can be
     * reached, the distance will be -1.
     */
    void recalculateDistances(std::vector<std::vector<int>> &grid,
            std::vector<GridPosition> targets);

    /** Returns a random action */
    std::unique_ptr<RockSampleAction> getRandomAction();

    /** Returns a random action from one of the given bins. */
    std::unique_ptr<RockSampleAction> getRandomAction(std::vector<long> binNumbers);

    /** Generates a random position within the problem space. */
    GridPosition samplePosition();
    /** Generates the state of the rocks uniformly at random. */
    std::vector<bool> sampleRocks();
    /** Decodes rocks from an integer. */
    std::vector<bool> decodeRocks(long val);
    /** Encodes rocks to an integer. */
    long encodeRocks(std::vector<bool> rockStates);

    /** The RockSampleOptions instance associated with this model. */
    RockSampleOptions *options_;

    /** The reward for sampling a good rock. */
    double goodRockReward_;
    /** The penalty for sampling a bad rock. */
    double badRockPenalty_;
    /** The reward for exiting the mapD. */
    double exitReward_;
    /** The penalty for an illegal move. */
    double illegalMovePenalty_;
    /** The half efficiency distance d0 */
    double halfEfficiencyDistance_;

    /** The number of rows in the map. */
    long nRows_;
    /** The number of columns in the map. */
    long nCols_;
    /** The number of rocks on the map. */
    long nRocks_;
    /** The starting position. */
    GridPosition startPos_;
    /** The coordinates of the rocks. */
    std::vector<GridPosition> rockPositions_;
    /** The coordinates of the goal squares. */
    std::vector<GridPosition> goalPositions_;

    /** The environment map in text form. */
    std::vector<std::string> mapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<RSCellType>> envMap_;


    /** The distance from each cell to the nearest goal square. */
    std::vector<std::vector<int>> goalDistances_;
    /** The distance from each cell to each rock. */
    std::vector<std::vector<std::vector<int>>> rockDistances_;

    /** The type of heuristic to use. (none / legal / preferred) */
    RSActionCategory heuristicType_;
    /** The category for search actions. */
    RSActionCategory searchCategory_;
    /** The category for rollout actions. */
    RSActionCategory rolloutCategory_;

    /** True iff we're initialising preferred actions with higher q-values. */
    bool usingPreferredInit_;
    /** The initial q-value for preferred actions. */
    double preferredQValue_;
    /** The initial visit count for preferred actions. */
    long preferredVisitCount_;

    /** Solver for the MDP version of the problem. */
    std::unique_ptr<RockSampleMdpSolver> mdpSolver_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_MODEL_HPP_ */
