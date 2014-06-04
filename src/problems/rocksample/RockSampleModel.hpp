#ifndef ROCKSAMPLE_MODEL_HPP_
#define ROCKSAMPLE_MODEL_HPP_

#include <ios>                          // for ostream
#include <memory>                       // for unique_ptr
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for variables_map

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"       // for State

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/enumerated_observations.hpp"

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model

#include "legal_actions.hpp"
#include "preferred_actions.hpp"
#include "RockSampleMdpSolver.hpp"

#include "global.hpp"                     // for RandomGenerator

namespace po = boost::program_options;

namespace solver {
class ActionMapping;
class StatePool;
class DiscretizedPoint;
} /* namespace solver */

namespace rocksample {
class RockSampleAction;
class RockSampleMdpSolver;
class RockSampleState;
class RockSampleObservation;

class RockSampleModel : virtual public ModelWithProgramOptions,
    virtual public PreferredActionsModel,
    virtual public solver::ModelWithEnumeratedObservations {
friend class RockSampleMdpSolver;
  public:
    RockSampleModel(RandomGenerator *randGen, po::variables_map vm);
    ~RockSampleModel() = default;
    _NO_COPY_OR_MOVE(RockSampleModel);

    /**
     * Rocks are enumerated 0, 1, 2, ... ;
     * other cell types should be negative.
     */
    enum RSCellType : int {
        ROCK = 0,
        EMPTY = -1,
        GOAL = -2,
    };

    /** Returns the name of this problem. */
    virtual std::string getName() override {
        return "RockSample";
    }
    /** Returns the number of rocks used in this model. */
    long getNumberOfRocks() {
        return nRocks_;
    }
    /** Returns true if only legal actions should be used. */
    bool usingOnlyLegal() {
        return usingOnlyLegal_;
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

    /** Returns the starting position for this problem. */
    GridPosition getStartPosition() {
        return startPos_;
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


    /***** Start implementation of Model's methods *****/
    // Simple getters
    virtual long getNumberOfStateVariables() override {
        return nStVars_;
    }
    virtual double getMinVal() override {
        return minVal_;
    }
    virtual double getMaxVal() override {
        return maxVal_;
    }

    // Other methods
    virtual std::unique_ptr<solver::State> sampleAnInitState() override;
    /** Generates a state uniformly at random. */
    virtual std::unique_ptr<solver::State> sampleStateUniform() override;

    virtual bool isTerminal(solver::State const &state) override;
    virtual double getHeuristicValue(solver::State const &state) override;


    /* --------------- Black box dynamics ----------------- */
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


    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action, solver::Observation const &obs,
            long nParticles,
            std::vector<solver::State const *> const &previousParticles) override;
    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs,
            long nParticles) override;

    /** Displays an individual cell of the map. */
    virtual void dispCell(RSCellType cellType, std::ostream &os);

    virtual void drawEnv(std::ostream &os) override;
    virtual void drawSimulationState(solver::BeliefNode const *belief,
            solver::State const &state,
            std::ostream &os) override;

    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>> getAllActionsInOrder();
    virtual std::vector<std::unique_ptr<solver::DiscretizedPoint>>
    getAllObservationsInOrder() override;

    virtual std::unique_ptr<solver::Serializer> createSerializer(solver::Solver *solver) override;

    /* ----------- Non-virtual methods for RockSampleModel ------------- */
    /** Generates the next position for the given position and action. */
    std::pair<GridPosition, bool> makeNextPosition(GridPosition pos,
            RockSampleAction const &action);
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

    /** Generates a random position within the problem space. */
    GridPosition samplePosition();
    /** Generates the state of the rocks uniformly at random. */
    std::vector<bool> sampleRocks();
    /** Decodes rocks from an integer. */
    std::vector<bool> decodeRocks(long val);
    /** Encodes rocks to an integer. */
    long encodeRocks(std::vector<bool> rockStates);

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

    /** The environment map in text form. */
    std::vector<std::string> mapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<RSCellType>> envMap_;

    /** True iff we're using only legal actions. */
    bool usingOnlyLegal_;

    /** True iff we're initialising preferred actions with higher q-values. */
    bool usingPreferredInit_;
    /** The initial q-value for preferred actions. */
    double preferredQValue_;
    /** The initial visit count for preferred actions. */
    long preferredVisitCount_;

    // Generic problem parameters
    long nStVars_;
    double minVal_, maxVal_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_MODEL_HPP_ */
