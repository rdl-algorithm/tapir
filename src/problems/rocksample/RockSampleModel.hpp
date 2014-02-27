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

#include "solver/geometry/Action.hpp"            // for Action
#include "solver/geometry/Observation.hpp"       // for Observation
#include "solver/geometry/State.hpp"       // for State

#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/enumerated_observations.hpp"

#include "solver/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/Model.hpp"             // for Model::StepResult, Model


#include "global.hpp"                     // for RandomGenerator

namespace po = boost::program_options;

namespace solver {
class ActionMapping;
class StatePool;
class EnumeratedPoint;
} /* namespace solver */

namespace rocksample {
class RockSampleAction;
class RockSampleState;
class RockSampleObservation;

class RockSampleModel : virtual public ModelWithProgramOptions,
    virtual public solver::ModelWithEnumeratedActions,
    virtual public solver::ModelWithEnumeratedObservations {
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

    virtual std::string getName() override {
        return "RockSample";
    }

    /***** Start implementation of Model's methods *****/
    // Simple getters
    virtual long getNStVars() override {
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
    virtual double getDefaultVal() override;

    /* --------------- Black box dynamics ----------------- */
    virtual std::unique_ptr<solver::State> generateNextState(
            solver::State const &state, solver::Action const &action) override;
    virtual std::unique_ptr<solver::Observation> generateObservation(
            solver::Action const &action,
            solver::State const &nextState) override;
    virtual double getReward(solver::State const &state,
                solver::Action const &action,
                solver::State const *nextState) override;
    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action) override;

    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action, solver::Observation const &obs,
            std::vector<solver::State const *> const &previousParticles) override;
    virtual std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::BeliefNode *previousBelief,
            solver::Action const &action,
            solver::Observation const &obs) override;

    virtual std::vector<long> loadChanges(char const *changeFilename) override;
    virtual void update(long time, solver::StatePool *pool) override;

    /** Displays an individual cell of the map. */
    virtual void dispCell(RSCellType cellType, std::ostream &os);

    virtual void drawEnv(std::ostream &os) override;
    virtual void drawState(solver::State const &state,
            std::ostream &os) override;

    virtual std::vector<std::unique_ptr<solver::EnumeratedPoint>>
    getAllActionsInOrder() override;
    virtual std::vector<std::unique_ptr<solver::EnumeratedPoint>>
    getAllObservationsInOrder() override;

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

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    std::pair<std::unique_ptr<RockSampleState>, bool> makeNextState(
            RockSampleState const &state, solver::Action const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    std::unique_ptr<RockSampleObservation> makeObservation(
            solver::Action const &action,
            RockSampleState const &nextState);
    /** Retrieves the reward via the next state. */
    double makeReward(RockSampleState const &state,
            solver::Action const &action, RockSampleState const &nextState,
            bool isLegal);

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

    // Generic problem parameters
    long nStVars_;
    double minVal_, maxVal_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_MODEL_HPP_ */
