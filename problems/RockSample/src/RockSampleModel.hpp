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

#include "ChangeType.hpp"               // for ChangeType
#include "GridPosition.hpp"             // for GridPosition
#include "Model.hpp"                    // for Model
#include "Observation.hpp"              // for Observation
#include "RockSampleState.hpp"          // for RockSampleState

namespace po = boost::program_options;

class RockSampleModel: public Model {
public:
    RockSampleModel(po::variables_map vm);
    ~RockSampleModel() = default;
    RockSampleModel(const RockSampleModel&) = delete;
    RockSampleModel(RockSampleModel&&) = delete;
    RockSampleModel &operator=(const RockSampleModel&) = delete;
    RockSampleModel &operator=(RockSampleModel&&) = delete;

    /**
     * Enumerates the possible actions. Note that there are actually
     * multiple check actions; Check-i is represented by CHECK+i,
     * where i is the rock number from 0..k-1 and k is the number
     * of rocks.
     */
    enum Action
        : long {
            NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3, SAMPLE = 4, CHECK = 5
    };

    void dispAct(unsigned long action, std::ostream &os) {
        if (action >= CHECK) {
            os << "CHECK-" << action - CHECK;
            return;
        }
        switch (action) {
        case NORTH:
            os << "NORTH";
            break;
        case EAST:
            os << "EAST";
            break;
        case SOUTH:
            os << "SOUTH";
            break;
        case WEST:
            os << "WEST";
            break;
        case SAMPLE:
            os << "SAMPLE";
            break;
        default:
            os << "ERROR-" << action;
            break;
        }
    }

    /**
     * There are only two possible observations - the rock
     * is either good or bad. Note that observations are
     * only meaningful when the action taken was CHECK;
     * they are meaningless otherwise.
     */
    enum Obs
        : int {
            NONE = 0, BAD = 1, GOOD = 2
    };

    /**
     * Rocks are enumerated 0, 1, 2, ... ;
     * other cell types should be negative.
     */
    enum CellType
        : int {
            ROCK = 0, EMPTY = -1, GOAL = -2,
    };

    void dispCell(int cellType, std::ostream &os) {
        if (cellType >= ROCK) {
            os << std::hex << cellType - ROCK;
            os << std::dec;
            return;
        }
        switch (cellType) {
        case EMPTY:
            os << '.';
            break;
        case GOAL:
            os << 'G';
            break;
        default:
            os << "ERROR-" << cellType;
            break;
        }
    }

    void dispObs(Observation &o, std::ostream &os) {
        switch ((int) o[0]) {
        case NONE:
            os << "NONE";
            break;
        case GOOD:
            os << "GOOD";
            break;
        case BAD:
            os << "BAD";
            break;
        default:
            os << "ERROR-" << o[0];
            break;
        }
    }

    /***** Start implementation of Model's virtual methods *****/
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

    // Other virtual methods
    std::unique_ptr<State> sampleAnInitState();
    bool isTerm(State &state);
    double solveHeuristic(State &state, double *qVal);
    double getDefaultVal();

    Model::StepResult generateStep(State &state, unsigned long action);
    double getReward(State &state);
    double getReward(State &state, unsigned long action);

    std::vector<std::unique_ptr<State>> generateParticles(unsigned long action, Observation &obs,
            std::vector<State*> &previousParticles);
    std::vector<std::unique_ptr<State>> getStatesSeeObs(unsigned long action, Observation &obs);

    std::vector<long> loadChanges(const char *changeFilename);
    void update(long tCh, std::vector<std::unique_ptr<State> > &affectedRange,
                std::vector<ChangeType> &typeOfChanges) = 0;

    virtual bool modifStSeq(std::vector<State*> &states,
                long startAffectedIdx, long endAffectedIdx,
                std::vector<std::unique_ptr<State> > &modifStSeq, std::vector<long> &modifActSeq,
                std::vector<Observation> &modifObsSeq,
                std::vector<double> &modifRewSeq) = 0;

    virtual void drawEnv(std::ostream &os) = 0;
    virtual void drawState(State &s, std::ostream &os) = 0;

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
    std::unique_ptr<State>  sampleStateUniform();
    /** Generates the state of the rocks uniformly at random. */
    std::vector<bool> sampleRocks(State &state);
    /** Decodes rocks from an integer. */
    std::vector<bool> decodeRocks(long val, State &state);

    /**
     * Generates a next state for the given state and action;
     * returns true if the action was legal, and false if it was illegal.
     */
    bool makeNextState(RockSampleState &state, long action, RockSampleState &nextState);
    /** Generates an observation given a next state (i.e. after the action)
     * and an action.
     */
    int makeObs(RockSampleState &nextState, long action);

    /** The number of rows in the map. */
    long nRows;
    /** The number of columns in the map. */
    long nCols;
    /** The number of rocks on the map. */
    long nRocks;
    /** The starting position. */
    GridPosition startPos;
    /** The coordinates of the rocks. */
    std::vector<GridPosition> rockGridPosition;

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
    std::vector<std::vector<int> > envMap;
};

#endif /* ROCKSAMPLE_MODEL_HPP */
