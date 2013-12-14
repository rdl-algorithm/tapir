#ifndef ROCKSAMPLE_MODEL_HPP_
#define ROCKSAMPLE_MODEL_HPP_

#include <ios>                          // for ostream
#include <memory>                       // for unique_ptr
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for variables_map

#include "defs.hpp"                     // for RandomGenerator
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions
#include "solver/Action.hpp"            // for Action
#include "solver/ChangeType.hpp"        // for ChangeType
#include "solver/Model.hpp"             // for Model::StepResult, Model
#include "solver/Observation.hpp"       // for Observation

namespace po = boost::program_options;

namespace solver {
class State;
} /* namespace solver */

namespace rocksample {
class RockSampleState;

class RockSampleModel : public ModelWithProgramOptions {
  public:
    RockSampleModel(RandomGenerator *randGen, po::variables_map vm);
    ~RockSampleModel() = default;
    RockSampleModel(RockSampleModel const &) = delete;
    RockSampleModel(RockSampleModel &&) = delete;
    RockSampleModel &operator=(RockSampleModel const &) = delete;
    RockSampleModel &operator=(RockSampleModel &&) = delete;

    /**
     * Enumerates the possible actions. Note that there are actually
     * multiple check actions; Check-i is represented by CHECK+i,
     * where i is the rock number from 0..k-1 and k is the number
     * of rocks.
     */
    enum RSAction : long {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3,
        SAMPLE = 4,
        CHECK = 5
    };

    /**
     * There are only two possible observations - the rock
     * is either good or bad. Note that observations are
     * only meaningful when the action taken was CHECK;
     * they are meaningless otherwise.
     */
    enum class RSObservation : unsigned int {
        NONE = 0,
        BAD = 1,
        GOOD = 2
    };

    /**
     * Rocks are enumerated 0, 1, 2, ... ;
     * other cell types should be negative.
     */
    enum RSCellType : int {
        ROCK = 0,
        EMPTY = -1,
        GOAL = -2,
    };

    /***** Start implementation of Model's methods *****/
    // Simple getters
    unsigned long getNActions() {
        return nActions_;
    }
    unsigned long getNObservations() {
        return nObservations_;
    }
    unsigned long getNStVars() {
        return nStVars_;
    }
    double getMinVal() {
        return minVal_;
    }
    double getMaxVal() {
        return maxVal_;
    }

    // Other methods
    std::unique_ptr<solver::State> sampleAnInitState();
    /** Generates a state uniformly at random. */
    std::unique_ptr<solver::State> sampleStateUniform();

    bool isTerm(solver::State const &state);
    double solveHeuristic(solver::State const &state);
    double getDefaultVal();

    solver::Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action);
    double getReward(solver::State const &state);
    double getReward(solver::State const &state, solver::Action const &action);

    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action, solver::Observation const &obs,
            std::vector<solver::State *> const &previousParticles);
    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action, solver::Observation const &obs);

    std::vector<long> loadChanges(char const *changeFilename);
    void update(long time,
            std::vector<std::unique_ptr<solver::State>> *affectedRange,
            std::vector<solver::ChangeType> *typeOfChanges);

    bool modifStSeq(std::vector<solver::State const *> const &states,
            long startAffectedIdx,
            long endAffectedIdx,
            std::vector<std::unique_ptr<solver::State>> *modifStSeq,
            std::vector<solver::Action> *modifActSeq,
            std::vector<solver::Observation> *modifObsSeq,
            std::vector<double> *modifRewSeq);

    void dispAct(solver::Action const &action, std::ostream &os);
    /** Displays an individual cell of the map. */
    void dispCell(RSCellType cellType, std::ostream &os);
    void dispObs(solver::Observation const &obs, std::ostream &os);
    void drawEnv(std::ostream &os);
    void drawState(solver::State const &state, std::ostream &os);

  private:
    // Problem parameters
    unsigned long nActions_, nObservations_, nStVars_;
    double minVal_, maxVal_;

    /**
     * Finds and counts the rocks on the map, and initialises the required
     * data structures and variables.
     */
    void initialise();

    /** Generates a random position within the problem space. */
    GridPosition samplePosition();
    /** Generates the state of the rocks uniformly at random. */
    std::vector<bool> sampleRocks();
    /** Decodes rocks from an integer. */
    std::vector<bool> decodeRocks(unsigned long val);
    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    std::pair<std::unique_ptr<RockSampleState>, bool> makeNextState(
            RockSampleState const &state, solver::Action const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    RSObservation makeObs(solver::Action const &action,
            RockSampleState const &nextState);

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

    /** The reward for sampling a good rock. */
    double goodRockReward_;
    /** The penalty for sampling a bad rock. */
    double badRockPenalty_;
    /** The reward for exiting the map. */
    double exitReward_;
    /** The penalty for an illegal move. */
    double illegalMovePenalty_;

    /** The half efficiency distance d0 */
    double halfEfficiencyDistance_;

    /** The environment map in text form. */
    std::vector<std::string> mapText_;
    /** The environment map in vector form. */
    std::vector<std::vector<RSCellType>> envMap_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_MODEL_HPP_ */
