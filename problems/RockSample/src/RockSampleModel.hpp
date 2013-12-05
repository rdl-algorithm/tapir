#ifndef ROCKSAMPLEMODEL_HPP
#define ROCKSAMPLEMODEL_HPP

#include <cstddef>                      // for size_t
#include <cstdlib>                      // for abs

#include <algorithm>                    // for copy
#include <ios>                          // for dec, hex
#include <iterator>                     // for ostream_iterator
#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream, operator<<, basic_ostream, basic_ostream::operator<<, basic_ostream<>::__ostream_type
#include <string>                       // for string
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for program_options, variables_map

#include "defs.hpp"                     // for RandomGenerator
#include "Action.hpp"                   // for Action
#include "ChangeType.hpp"               // for ChangeType
#include "GridPosition.hpp"             // for GridPosition
#include "Model.hpp"                    // for Model
#include "Observation.hpp"              // for Observation
#include "RockSampleState.hpp"          // for RockSampleState

namespace po = boost::program_options;

class RockSampleModel: public Model {
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
    enum CellType : int {
        ROCK = 0,
        EMPTY = -1,
        GOAL = -2,
    };

    /***** Start implementation of Model's methods *****/
    // Simple getters
    double getDiscount() {
        return discount;
    }
    unsigned long getNActions() {
        return nActions;
    }
    unsigned long getNObservations() {
        return nObservations;
    }
    unsigned long getNStVars() {
        return nStVars;
    }
    double getMinVal() {
        return minVal;
    }
    double getMaxVal() {
        return maxVal;
    }

    unsigned long getNParticles() {
        return nParticles;
    }
    long getMaxTrials() {
        return maxTrials;
    }
    double getDepthTh() {
        return depthTh;
    }
    double getExploreCoef() {
        return exploreCoef;
    }
    long getMaxDistTry() {
        return maxDistTry;
    }
    double getDistTh() {
        return distTh;
    }

    // Other methods
    std::unique_ptr<State> sampleAnInitState();
    bool isTerm(State const &state);
    double solveHeuristic(State const &state);
    double getDefaultVal();

    Model::StepResult generateStep(State const &state, Action const &action);
    double getReward(State const &state);
    double getReward(State const &state, Action const &action);

    std::vector<std::unique_ptr<State>> generateParticles(Action const &action,
                                     Observation const &obs,
                                     std::vector<State *> const &previousParticles);
    std::vector<std::unique_ptr<State>> generateParticles(Action const &action,
                                     Observation const &obs);

    std::vector<long> loadChanges(char const *changeFilename);
    void update(long time, std::vector<std::unique_ptr<State> > *affectedRange,
                std::vector<ChangeType> *typeOfChanges);

    bool modifStSeq(std::vector<State const *> const &states, long startAffectedIdx,
                    long endAffectedIdx,
                    std::vector<std::unique_ptr<State> > *modifStSeq,
                    std::vector<Action> *modifActSeq,
                    std::vector<Observation> *modifObsSeq,
                    std::vector<double> *modifRewSeq);

    void dispAct(Action const &action, std::ostream &os);
    void dispCell(CellType cellType, std::ostream &os);
    void dispObs(Observation const &obs, std::ostream &os);
    void drawEnv(std::ostream &os);
    void drawState(State const &state, std::ostream &os);

  private:
    // Problem parameters.
    double discount;
    unsigned long nActions, nObservations, nStVars;
    double minVal, maxVal;

    // SBT parameters
    unsigned long nParticles;
    long maxTrials;
    double depthTh;
    double exploreCoef;

    long maxDistTry;
    double distTh;

    /**
     * Finds and counts the rocks on the map, and initialises the required
     * data structures and variables.
     */
    void initialise();

    /** Generates a state uniformly at random. */
    std::unique_ptr<State> sampleStateUniform();

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
        RockSampleState const &state, Action const &action);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    RSObservation makeObs(Action const &action, RockSampleState const &nextState);

    /** The number of rows in the map. */
    long nRows;
    /** The number of columns in the map. */
    long nCols;
    /** The number of rocks on the map. */
    long nRocks;
    /** The starting position. */
    GridPosition startPos;
    /** The coordinates of the rocks. */
    std::vector<GridPosition> rockPositions;

    /** The reward for sampling a good rock. */
    double goodRockReward;
    /** The penalty for sampling a bad rock. */
    double badRockPenalty;
    /** The reward for exiting the map. */
    double exitReward;
    /** The penalty for an illegal move. */
    double illegalMovePenalty;

    /** The half efficiency distance d0 */
    double halfEfficiencyDistance;

    /** The environment map in text form. */
    std::vector<std::string> mapText;
    /** The environment map in vector form. */
    std::vector<std::vector<CellType> > envMap;
};

#endif /* ROCKSAMPLE_MODEL_HPP */
