#include "RockSampleModel.hpp"

#include <cmath>                        // for pow, floor
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <fstream>                      // for operator<<, basic_ostream, endl, basic_ostream<>::__ostream_type, ifstream, basic_ostream::operator<<, basic_istream, basic_istream<>::__istream_type
#include <iostream>                     // for cout, cerr
#include <memory>                       // for unique_ptr, default_delete
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <set>                          // for set, _Rb_tree_const_iterator, set<>::iterator
#include <string>                       // for string, getline, char_traits, basic_string
#include <tuple>                        // for tie, tuple
#include <unordered_map>                // for unordered_map<>::value_type, unordered_map
#include <utility>                      // for move, pair, make_pair
#include <vector>                       // for vector, vector<>::reference, __alloc_traits<>::value_type, operator==

#include <boost/program_options.hpp>    // for variables_map, variable_value

#include "defs.hpp"                     // for RandomGenerator, make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator<<
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions
#include "solver/Action.hpp"            // for Action
#include "solver/ChangeType.hpp"        // for ChangeType
#include "solver/Model.hpp"             // for Model::StepResult, Model
#include "solver/Observation.hpp"       // for Observation
#include "solver/State.hpp"             // for State, operator<<, State::Hash, operator==

#include "RockSampleState.hpp"          // for RockSampleState

using std::cerr;
using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace rocksample {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
RockSampleModel::RockSampleModel(RandomGenerator *randGen,
        po::variables_map vm) : ModelWithProgramOptions(randGen, vm),
    goodRockReward_(vm["problem.goodRockReward"].as<double>()),
    badRockPenalty_(vm["problem.badRockPenalty"].as<double>()),
    exitReward_(vm["problem.exitReward"].as<double>()),
    illegalMovePenalty_(
            vm["problem.illegalMovePenalty"].as<double>()),
    halfEfficiencyDistance_(
            vm["problem.halfEfficiencyDistance"].as<double>()) {
#pragma GCC diagnostic pop
    // Read the map from the file.
    std::ifstream inFile;
    char const *mapPath = vm["problem.mapPath"].as<std::string>().c_str();
    inFile.open(mapPath);
    if (!inFile.is_open()) {
        std::cerr << "Fail to open " << mapPath << "\n";
        exit(1);
    }
    inFile >> nRows_ >> nCols_;
    std::string tmp;
    getline(inFile, tmp);
    for (long i = 0; i < nRows_; i++) {
        getline(inFile, tmp);
        mapText_.push_back(tmp);
    }
    inFile.close();

    initialise();
    cout << "Constructed the RockSampleModel" << endl;
    cout << "Discount: " << getDiscountFactor() << endl;
    cout << "Size: " << nRows_ << " by " << nCols_ << endl;
    cout << "Start: " << startPos_.i << " " << startPos_.j << endl;
    cout << "nRocks: " << nRocks_ << endl;
    cout << "Rock 0: " << rockPositions_[0] << endl;
    cout << "Rock 1: " << rockPositions_[1] << endl;
    cout << "good rock reward: " << goodRockReward_ << endl;
    cout << "nActions: " << nActions_ << endl;
    cout << "nObservations: " << nObservations_ << endl;
    cout << "nStVars: " << nStVars_ << endl;
    cout << "Random initial states:" << endl;
    cout << *sampleAnInitState() << endl;
    cout << *sampleAnInitState() << endl;
    cout << *sampleAnInitState() << endl;
    cout << *sampleAnInitState() << endl;

    cout << "nParticles: " << getNParticles() << endl;
    cout << "Environment:" << endl;
    drawEnv(cout);
}

void RockSampleModel::initialise() {
    nRocks_ = 0;
    GridPosition p;
    for (p.i = 0; p.i < nRows_; p.i++) {
        envMap_.emplace_back();
        for (p.j = 0; p.j < nCols_; p.j++) {
            char c = mapText_[p.i][p.j];
            RSCellType cellType;
            if (c == 'o') {
                rockPositions_.push_back(p);
                cellType = (RSCellType)(ROCK + nRocks_);
                nRocks_++;
            } else if (c == 'G') {
                cellType = GOAL;
            } else if (c == 'S') {
                startPos_ = p;
                cellType = EMPTY;
            } else {
                cellType = EMPTY;
            }
            envMap_.back().push_back(cellType);
        }
    }

    nActions_ = 5 + nRocks_;
    nObservations_ = 2;
    nStVars_ = 2 + nRocks_;
    minVal_ = -illegalMovePenalty_ / (1 - getDiscountFactor());
    maxVal_ = goodRockReward_ * nRocks_ + exitReward_;
}

std::unique_ptr<solver::State> RockSampleModel::sampleAnInitState() {
    return std::make_unique<RockSampleState>(startPos_, sampleRocks());
}

std::unique_ptr<solver::State> RockSampleModel::sampleStateUniform() {
    return std::make_unique<RockSampleState>(samplePosition(), sampleRocks());
}

GridPosition RockSampleModel::samplePosition() {
    unsigned long i = std::uniform_int_distribution<unsigned long>(
                0, nRows_ - 1)(*randGen_);
    unsigned long j = std::uniform_int_distribution<unsigned long>(
                0, nCols_ - 1)(*randGen_);
    return GridPosition(i, j);
}

std::vector<bool> RockSampleModel::sampleRocks() {
    return decodeRocks(std::uniform_int_distribution<unsigned long>
                (0, (1 << nRocks_) - 1)(*randGen_));
}

std::vector<bool> RockSampleModel::decodeRocks(unsigned long val) {
    std::vector<bool> isRockGood;
    for (int j = 0; j < nRocks_; j++) {
        isRockGood.push_back(val &  (1 << j));
    }
    return isRockGood;
}

bool RockSampleModel::isTerm(solver::State const &state) {
    RockSampleState const *rockSampleState =
        static_cast<RockSampleState const *>(&state);
    GridPosition pos = rockSampleState->getPosition();
    return envMap_[pos.i][pos.j] == GOAL;
}

double RockSampleModel::solveHeuristic(solver::State const &state) {
    RockSampleState const *rockSampleState =
        static_cast<RockSampleState const *>(&state);
    double qVal = 0;
    double currentDiscount = 1;
    GridPosition currentPos(rockSampleState->getPosition());
    std::vector<bool> rockStates(rockSampleState->getRockStates());

    std::set<int> goodRocks;
    for (int i = 0; i < nRocks_; i++) {
        if (rockStates[i]) {
            goodRocks.insert(i);
        }
    }
    while (!goodRocks.empty()) {
        std::set<int>::iterator it = goodRocks.begin();
        int bestRock = *it;
        long lowestDist =
            rockPositions_[bestRock].manhattanDistanceTo(currentPos);
        ++it;
        for (; it != goodRocks.end(); ++it) {
            long dist = rockPositions_[*it].manhattanDistanceTo(currentPos);
            if (dist < lowestDist) {
                bestRock = *it;
                lowestDist = dist;
            }
        }
        currentDiscount *= std::pow(getDiscountFactor(), lowestDist);
        qVal += currentDiscount * goodRockReward_;
        goodRocks.erase(bestRock);
        currentPos = rockPositions_[bestRock];
    }
    currentDiscount *= std::pow(getDiscountFactor(), nCols_ - currentPos.j);
    qVal += currentDiscount * exitReward_;
    // dispState(s, cerr);
    // cerr << endl << "Heuristic: " << *qVal << endl;
    return qVal;
}

double RockSampleModel::getDefaultVal() {
    return minVal_;
}

std::pair<std::unique_ptr<RockSampleState>,
        bool> RockSampleModel::makeNextState(
        RockSampleState const &state, solver::Action const &action) {
    GridPosition pos(state.getPosition());
    std::vector<bool> rockStates(state.getRockStates());
    bool isValid = true;
    if (action >= CHECK + nRocks_) {
        cerr << "Invalid action: " << action << endl;
    } else if (action >= CHECK) {
        // Do nothing - the state remains the same.
    } else if (action == SAMPLE) {
        int rockNo = envMap_[pos.i][pos.j] - ROCK;
        if (0 <= rockNo && rockNo < nRocks_) {
            rockStates[rockNo] = false;
        } else {
            // cerr << "Cannot sample at " << pos << " - no rock!" << endl;
            isValid = false;
        }
    } else {
        if (action == NORTH) {
            pos.i -= 1;
        } else if (action == EAST) {
            pos.j += 1;
        } else if (action == SOUTH) {
            pos.i += 1;
        } else if (action == WEST) {
            pos.j -= 1;
        } else {
            cerr << "Invalid action: " << action << endl;
        }
        // If the position is now invalid, reset it.
        if (pos.i < 0 || pos.i >= nRows_ || pos.j < 0 || pos.j >= nCols_) {
            pos = state.getPosition();
            isValid = false;
        }
    }
    return std::make_pair(std::make_unique<RockSampleState>(pos, rockStates),
            isValid);
}

RockSampleModel::RSObservation RockSampleModel::makeObs(
        solver::Action const &action,
        RockSampleState const &nextState) {
    if (action < CHECK) {
        return RSObservation::NONE;
    }
    int rockNo = action - CHECK;
    GridPosition pos(nextState.getPosition());
    std::vector<bool> rockStates(nextState.getRockStates());
    double dist = pos.euclideanDistanceTo(rockPositions_[rockNo]);
    double efficiency =
        (1 + std::pow(2, -dist / halfEfficiencyDistance_)) * 0.5;
    // cerr << "D: " << dist << " E:" << efficiency << endl;
    if (std::bernoulli_distribution(efficiency)(*randGen_)) {
        return rockStates[rockNo] ? RSObservation::GOOD : RSObservation::BAD; // Correct obs.
    } else {
        return rockStates[rockNo] ? RSObservation::BAD : RSObservation::GOOD; // Incorrect obs.
    }
}

solver::Model::StepResult RockSampleModel::generateStep(
        solver::State const &state,
        solver::Action const &action) {
    RockSampleState const *rockSampleState =
        static_cast<RockSampleState const *>(&state);
    solver::Model::StepResult result;
    result.action = action;

    std::unique_ptr<RockSampleState> nextState = makeNextState(
                *rockSampleState, action).first;

    result.observation.push_back((double)makeObs(action, *nextState));
    result.immediateReward = getReward(state, action);
    result.isTerminal = isTerm(*nextState);
    result.nextState = std::move(nextState);
    return result;
}

double RockSampleModel::getReward(solver::State const & /*state*/) {
    return 0;
}

double RockSampleModel::getReward(solver::State const &state,
        solver::Action const &action) {
    RockSampleState const *rockSampleState =
        static_cast<RockSampleState const *>(&state);

    std::unique_ptr<RockSampleState> nextState;
    bool isLegal;
    std::tie(nextState, isLegal) = makeNextState(*rockSampleState, action);

    if (!isLegal) {
        return -illegalMovePenalty_;
    }
    if (isTerm(*nextState)) {
        return exitReward_;
    }

    if (action == SAMPLE) {
        GridPosition pos = rockSampleState->getPosition();
        int rockNo = envMap_[pos.i][pos.j] - ROCK;
        if (0 <= rockNo && rockNo < nRocks_) {
            return rockSampleState->getRockStates()[rockNo] ? goodRockReward_
                   : -badRockPenalty_;
        } else {
            cerr << "Invalid sample action!?!" << endl;
            return -illegalMovePenalty_;
        }
    }
    return 0;
}

std::vector<std::unique_ptr<solver::State>> RockSampleModel::generateParticles(
        solver::Action const &action, solver::Observation const &obs,
        std::vector<solver::State *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    // If it's a CHECK action, we condition on the observation.
    if (action >= CHECK) {
        int rockNo = action - CHECK;
        typedef std::unordered_map<RockSampleState, double,
                solver::State::Hash> WeightMap;
        WeightMap weights;
        double weightTotal = 0;
        for (solver::State *state : previousParticles) {
            RockSampleState const *rockSampleState =
                static_cast<RockSampleState const *>(state);
            GridPosition pos(rockSampleState->getPosition());
            double dist = pos.euclideanDistanceTo(rockPositions_[rockNo]);
            double efficiency = ((1
                                  + std::pow(2, -dist
                                          / halfEfficiencyDistance_)) * 0.5);
            bool rockIsGood = rockSampleState->getRockStates()[rockNo];
            double probability;
            if ((rockIsGood && obs[0] == (double)RSObservation::GOOD)
                || (!rockIsGood && obs[0] == (double)RSObservation::BAD)) {
                probability = efficiency;
            } else {
                probability = 1 - efficiency;
            }
            weights[*rockSampleState] += probability;
            weightTotal += probability;
        }
        double scale = getNParticles() / weightTotal;
        for (WeightMap::value_type &it : weights) {
            double proportion = it.second * scale;
            int numToAdd = std::floor(proportion);
            if (std::bernoulli_distribution(proportion - numToAdd)(*randGen_)) {
                numToAdd += 1;
            }
            for (int i = 0; i < numToAdd; i++) {
                newParticles.push_back(std::make_unique<RockSampleState>(it.
                                first));
            }
        }

    } else {
        // It's not a CHECK action, so we just add each resultant state.
        for (solver::State *state : previousParticles) {
            RockSampleState const *rockSampleState =
                static_cast<RockSampleState const *>(state);
            newParticles.push_back(makeNextState(*rockSampleState,
                            action).first);
        }
    }
    return newParticles;
}

std::vector<std::unique_ptr<solver::State>> RockSampleModel::generateParticles(
        solver::Action const &action, solver::Observation const &obs) {
    std::vector<std::unique_ptr<solver::State>> particles;
    while (particles.size() < getNParticles()) {
        std::unique_ptr<solver::State> state = sampleStateUniform();
        solver::Model::StepResult result = generateStep(*state, action);
        if (obs == result.observation) {
            particles.push_back(std::move(result.nextState));
        }
    }
    return particles;
}

std::vector<long> RockSampleModel::loadChanges(char const */*changeFilename*/) {
    std::vector<long> result;
    return result;
}

void RockSampleModel::update(long /*time*/,
        std::vector<std::unique_ptr<solver::State>> */*affectedRange*/,
        std::vector<solver::ChangeType> */*typeOfChanges*/) {
}

bool RockSampleModel::modifStSeq(
        std::vector<solver::State const *> const & /*states*/,
        long /*startAffectedIdx*/, long /*endAffectedIdx*/,
        std::vector<std::unique_ptr<solver::State>> */*modifStSeq*/,
        std::vector<solver::Action> */*modifActSeq*/,
        std::vector<solver::Observation> */*modifObsSeq*/,
        std::vector<double> */*modifRewSeq*/) {
    return false;
}

void RockSampleModel::dispAct(solver::Action const &action, std::ostream &os) {
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

void RockSampleModel::dispCell(RSCellType cellType, std::ostream &os) {
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

void RockSampleModel::dispObs(solver::Observation const &obs,
        std::ostream &os) {
    switch ((int)obs[0]) {
    case (int)RSObservation::NONE:
        os << "NONE";
        break;
    case (int)RSObservation::GOOD:
        os << "GOOD";
        break;
    case (int)RSObservation::BAD:
        os << "BAD";
        break;
    default:
        os << "ERROR-" << obs[0];
        break;
    }
}

void RockSampleModel::drawEnv(std::ostream &os) {
    for (std::vector<RSCellType> &row : envMap_) {
        for (RSCellType cellValue : row) {
            dispCell(cellValue, os);
        }
        os << endl;
    }
}

void RockSampleModel::drawState(solver::State const &state, std::ostream &os) {
    RockSampleState const *rockSampleState =
        static_cast<RockSampleState const *>(&state);
    os << state << endl;
    GridPosition pos(rockSampleState->getPosition());
    for (std::size_t i = 0; i < envMap_.size(); i++) {
        for (std::size_t j = 0; j < envMap_[0].size(); j++) {
            if ((long)i == pos.i && (long)j == pos.j) {
                os << "x";
                continue;
            }
            dispCell(envMap_[i][j], os);
        }
        os << endl;
    }
}
} /* namespace rocksample */
