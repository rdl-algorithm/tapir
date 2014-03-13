#include "RockSampleModel.hpp"

#include <cmath>                        // for pow, floor
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <fstream>                      // for operator<<, basic_ostream, endl, basic_ostream<>::__ostream_type, ifstream, basic_ostream::operator<<, basic_istream, basic_istream<>::__istream_type
#include <initializer_list>
#include <iostream>                     // for cout
#include <map>
#include <memory>                       // for unique_ptr, default_delete
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <set>                          // for set, _Rb_tree_const_iterator, set<>::iterator
#include <string>                       // for string, getline, char_traits, basic_string
#include <tuple>                        // for tie, tuple
#include <unordered_map>                // for unordered_map<>::value_type, unordered_map
#include <utility>                      // for move, pair, make_pair
#include <vector>                       // for vector, vector<>::reference, __alloc_traits<>::value_type, operator==

#include <boost/program_options.hpp>    // for variables_map, variable_value

#include "global.hpp"                     // for RandomGenerator, make_unique

#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator<<
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions

#include "solver/abstract-problem/Action.hpp"            // for Action
#include "solver/abstract-problem/Model.hpp"             // for Model::StepResult, Model
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"       // for State

#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/enumerated_actions.hpp"
#include "solver/mappings/enumerated_observations.hpp"

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/StatePool.hpp"

#include "RockSampleAction.hpp"         // for RockSampleAction
#include "RockSampleObservation.hpp"    // for RockSampleObservation
#include "RockSampleState.hpp"          // for RockSampleState

using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace rocksample {
RockSampleModel::RockSampleModel(RandomGenerator *randGen,
        po::variables_map vm) :
    ModelWithProgramOptions(randGen, vm),
    ModelWithEnumeratedActions({}),
    goodRockReward_(vm["problem.goodRockReward"].as<double>()),
    badRockPenalty_(vm["problem.badRockPenalty"].as<double>()),
    exitReward_(vm["problem.exitReward"].as<double>()),
    illegalMovePenalty_(
            vm["problem.illegalMovePenalty"].as<double>()),
    halfEfficiencyDistance_(
            vm["problem.halfEfficiencyDistance"].as<double>()),
    nRows_(0), // update
    nCols_(0), // update
    nRocks_(0), // update
    startPos_(), // update
    rockPositions_(), // push rocks
    mapText_(), // push rows
    envMap_(), // push rows
    nStVars_(), // depends on nRocks
    minVal_(-illegalMovePenalty_ / (1 - getDiscountFactor())),
    maxVal_(0) // depends on nRocks
         {
    // Read the map from the file.
    std::ifstream inFile;
    char const *mapPath = vm["problem.mapPath"].as<std::string>().c_str();
    inFile.open(mapPath);
    if (!inFile.is_open()) {
        std::ostringstream message;
        message << "Failed to open " << mapPath;
        debug::show_message(message.str());
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

    initialize();
    cout << "Constructed the RockSampleModel" << endl;
    cout << "Discount: " << getDiscountFactor() << endl;
    cout << "Size: " << nRows_ << " by " << nCols_ << endl;
    cout << "Start: " << startPos_.i << " " << startPos_.j << endl;
    cout << "nRocks: " << nRocks_ << endl;
    cout << "Rock 0: " << rockPositions_[0] << endl;
    cout << "Rock 1: " << rockPositions_[1] << endl;
    cout << "good rock reward: " << goodRockReward_ << endl;
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

void RockSampleModel::initialize() {
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

    nStVars_ = 2 + nRocks_;
    minVal_ = -illegalMovePenalty_ / (1 - getDiscountFactor());
    maxVal_ = goodRockReward_ * nRocks_ + exitReward_;
    setAllActions(getAllActionsInOrder());
}

std::unique_ptr<solver::State> RockSampleModel::sampleAnInitState() {
    return std::make_unique<RockSampleState>(startPos_, sampleRocks());
}

std::unique_ptr<solver::State> RockSampleModel::sampleStateUniform() {
    return std::make_unique<RockSampleState>(samplePosition(), sampleRocks());
}

GridPosition RockSampleModel::samplePosition() {
    long i = std::uniform_int_distribution<long>(
                0, nRows_ - 1)(*getRandomGenerator());
    long j = std::uniform_int_distribution<long>(
                0, nCols_ - 1)(*getRandomGenerator());
    return GridPosition(i, j);
}

std::vector<bool> RockSampleModel::sampleRocks() {
    return decodeRocks(std::uniform_int_distribution<long>
                (0, (1 << nRocks_) - 1)(*getRandomGenerator()));
}

std::vector<bool> RockSampleModel::decodeRocks(long val) {
    std::vector<bool> isRockGood;
    for (int j = 0; j < nRocks_; j++) {
        isRockGood.push_back(val &  (1 << j));
    }
    return isRockGood;
}

bool RockSampleModel::isTerminal(solver::State const &state) {
    RockSampleState const &rockSampleState =
        static_cast<RockSampleState const &>(state);
    GridPosition pos = rockSampleState.getPosition();
    return envMap_[pos.i][pos.j] == GOAL;
}

double RockSampleModel::getHeuristicValue(solver::State const &state) {
    RockSampleState const &rockSampleState =
        static_cast<RockSampleState const &>(state);
    double qVal = 0;
    double currentDiscount = 1;
    GridPosition currentPos(rockSampleState.getPosition());
    std::vector<bool> rockStates(rockSampleState.getRockStates());

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
    RockSampleAction const &a = static_cast<RockSampleAction const &>(action);
    ActionType actionType = a.getActionType();
    if (actionType == ActionType::CHECK) {
        // Do nothing - the state remains the same.
    } else if (actionType == ActionType::SAMPLE) {
        int rockNo = envMap_[pos.i][pos.j] - ROCK;
        if (0 <= rockNo && rockNo < nRocks_) {
            rockStates[rockNo] = false;
        } else {
            // std::ostringstream message;
            // message << "Cannot sample at " << pos << " - no rock!";
            // debug::show_message(message.str());
            isValid = false;
        }
    } else {
        if (actionType == ActionType::NORTH) {
            pos.i -= 1;
        } else if (actionType == ActionType::EAST) {
            pos.j += 1;
        } else if (actionType == ActionType::SOUTH) {
            pos.i += 1;
        } else if (actionType == ActionType::WEST) {
            pos.j -= 1;
        } else {
            std::ostringstream message;
            message << "Invalid action: " << action;
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

std::unique_ptr<RockSampleObservation> RockSampleModel::makeObservation(
        solver::Action const &action,
        RockSampleState const &nextState) {
    RockSampleAction const &a = static_cast<RockSampleAction const &>(action);
    ActionType actionType = a.getActionType();
    if (actionType < ActionType::CHECK) {
        return std::make_unique<RockSampleObservation>();
    }
    long rockNo = a.getRockNo();
    GridPosition pos(nextState.getPosition());
    std::vector<bool> rockStates(nextState.getRockStates());
    double dist = pos.euclideanDistanceTo(rockPositions_[rockNo]);
    double efficiency =
        (1 + std::pow(2, -dist / halfEfficiencyDistance_)) * 0.5;
    bool obsMatches = std::bernoulli_distribution(efficiency)(*getRandomGenerator());
    return std::make_unique<RockSampleObservation>(rockStates[rockNo] == obsMatches);
}

double RockSampleModel::makeReward(RockSampleState const &state,
        solver::Action const &action, RockSampleState const &nextState,
        bool isLegal) {
    if (!isLegal) {
        return -illegalMovePenalty_;
    }
    if (isTerminal(nextState)) {
        return exitReward_;
    }

    ActionType actionType = static_cast<RockSampleAction const &>(action).getActionType();
    if (actionType == ActionType::SAMPLE) {
        GridPosition pos = state.getPosition();
        int rockNo = envMap_[pos.i][pos.j] - ROCK;
        if (0 <= rockNo && rockNo < nRocks_) {
            return state.getRockStates()[rockNo] ? goodRockReward_
                   : -badRockPenalty_;
        } else {
            debug::show_message("Invalid sample action!?!");
            return -illegalMovePenalty_;
        }
    }
    return 0;
}

std::unique_ptr<solver::State> RockSampleModel::generateNextState(
           solver::State const &state,
           solver::Action const &action,
           solver::TransitionParameters const */*tp*/) {
    return makeNextState(static_cast<RockSampleState const &>(state),
            action).first;
}

std::unique_ptr<solver::Observation> RockSampleModel::generateObservation(
        solver::State const */*state*/,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/,
        solver::State const &nextState) {
    return makeObservation(action,
            static_cast<RockSampleState const &>(nextState));
}

double RockSampleModel::generateReward(
        solver::State const &state,
        solver::Action const &action,
        solver::TransitionParameters const */*tp*/,
        solver::State const */*nextState*/) {
    RockSampleState const &rockSampleState = (
            static_cast<RockSampleState const &>(state));
    std::unique_ptr<RockSampleState> nextState;
    bool isLegal;
    std::tie(nextState, isLegal) = makeNextState(rockSampleState, action);
    return makeReward(rockSampleState, action, *nextState, isLegal);
}

solver::Model::StepResult RockSampleModel::generateStep(
        solver::State const &state,
        solver::Action const &action) {
    RockSampleState const &rockSampleState =
        static_cast<RockSampleState const &>(state);
    solver::Model::StepResult result;
    result.action = action.copy();

    bool isLegal;
    std::unique_ptr<RockSampleState> nextState;
    std::tie(nextState, isLegal) = makeNextState(rockSampleState, action);
    result.observation = makeObservation(action, *nextState);
    result.reward = makeReward(rockSampleState, action, *nextState, isLegal);
    result.isTerminal = isTerminal(*nextState);
    result.nextState = std::move(nextState);
    return result;
}

std::vector<std::unique_ptr<solver::State>> RockSampleModel::generateParticles(
        solver::BeliefNode */*previousBelief*/,
        solver::Action const &action, solver::Observation const &obs,
        std::vector<solver::State const *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;

    RockSampleAction const &a = static_cast<RockSampleAction const &>(action);
    if (a.getActionType() == ActionType::CHECK) {
        long rockNo = a.getRockNo();
        struct Hash {
            std::size_t operator()(RockSampleState const &state) const {
                return state.hash();
            }
        };
        typedef std::unordered_map<RockSampleState, double, Hash> WeightMap;
        WeightMap weights;
        double weightTotal = 0;
        for (solver::State const *state : previousParticles) {
            RockSampleState const *rockSampleState =
                static_cast<RockSampleState const *>(state);
            GridPosition pos(rockSampleState->getPosition());
            double dist = pos.euclideanDistanceTo(rockPositions_[rockNo]);
            double efficiency = ((1
                                  + std::pow(2, -dist
                                          / halfEfficiencyDistance_)) * 0.5);
            bool rockIsGood = rockSampleState->getRockStates()[rockNo];
            double probability;
            RockSampleObservation const &observation = (
                    static_cast<RockSampleObservation const &>(obs));
            if (rockIsGood == observation.isGood()) {
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
            long numToAdd = static_cast<long>(proportion);
            if (std::bernoulli_distribution(proportion - numToAdd)(
                    *getRandomGenerator())) {
                numToAdd += 1;
            }
            for (int i = 0; i < numToAdd; i++) {
                newParticles.push_back(std::make_unique<RockSampleState>(it.
                                first));
            }
        }

    } else {
        // It's not a CHECK action, so we just add each resultant state.
        for (solver::State const *state : previousParticles) {
            RockSampleState const *rockSampleState =
                static_cast<RockSampleState const *>(state);
            newParticles.push_back(makeNextState(*rockSampleState,
                            action).first);
        }
    }
    return newParticles;
}

std::vector<std::unique_ptr<solver::State>> RockSampleModel::generateParticles(
        solver::BeliefNode */*previousBelief*/,
        solver::Action const &action, solver::Observation const &obs) {
    std::vector<std::unique_ptr<solver::State>> particles;
    while (particles.size() < getNParticles()) {
        std::unique_ptr<solver::State> state = sampleStateUniform();
        solver::Model::StepResult result = generateStep(*state, action);
        if (obs == *result.observation) {
            particles.push_back(std::move(result.nextState));
        }
    }
    return particles;
}

std::vector<long> RockSampleModel::loadChanges(char const */*changeFilename*/) {
    std::vector<long> result;
    return result;
}

void RockSampleModel::update(long /*time*/, solver::StatePool */*pool*/) {
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

void RockSampleModel::drawEnv(std::ostream &os) {
    for (std::vector<RSCellType> &row : envMap_) {
        for (RSCellType cellValue : row) {
            dispCell(cellValue, os);
        }
        os << endl;
    }
}

void RockSampleModel::drawSimulationState(
        solver::BeliefNode *belief,
        solver::State const &state, std::ostream &os) {
    RockSampleState const &rockSampleState =
            static_cast<RockSampleState const &>(state);
    std::vector<solver::State const *> particles = belief->getStates();
    os << belief->getQValue();
    os << " from " << particles.size() << " particles." << endl;
    GridPosition pos(rockSampleState.getPosition());
    std::vector<double> goodProportions(nRocks_);
    for (solver::State const *particle : particles) {
        RockSampleState const &rss =
                static_cast<RockSampleState const &>(*particle);
        for (long i = 0; i < nRocks_; i++) {
            if (rss.getRockStates()[i]) {
                goodProportions[i] += 1;
            }
        }
    }
    for (long i = 0; i < nRocks_; i++) {
        goodProportions[i] /= particles.size();
    }
    for (std::size_t i = 0; i < envMap_.size(); i++) {
        for (std::size_t j = 0; j < envMap_[0].size(); j++) {
            long rockNo = envMap_[i][j] - ROCK;
            if (rockNo >= 0 && hasColorOutput()) {
                std::vector<int> colors {196,
                    161, 126, 91, 56, 21,
                    26, 31, 36, 41, 46};
                int color =
                        colors[goodProportions[rockNo] * (colors.size() - 1)];
                os << "\033[38;5;" << color << "m";
            }
            if ((long) i == pos.i && (long) j == pos.j) {
                os << "x";
            } else {
                dispCell(envMap_[i][j], os);
            }
            if (rockNo >= 0 && hasColorOutput()) {
                os << "\033[0m";
            }
        }
        os << endl;
    }
    os << "Action children: " << endl;
    std::multimap<double, solver::ActionMappingEntry const *> actionValues;
    for (solver::ActionMappingEntry const *entry : belief->getMapping()->getChildEntries()) {
        actionValues.emplace(entry->getActionNode()->getQValue(), entry);
    }
    for (auto it = actionValues.rbegin(); it != actionValues.rend(); it++) {
        abt::printDouble(it->first, os, false, 3, 3);
        os << ": ";
        std::ostringstream sstr;
        sstr << *it->second->getAction();
        abt::printWithWidth(sstr.str(), os, 7);
        abt::printWithWidth(it->second->getActionNode()->getNParticles(), os, 6);
        os << endl;
    }
}

std::vector<std::unique_ptr<solver::DiscretizedPoint>>
RockSampleModel::getAllActionsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allActions;
    for (long code = 0; code < 5 + nRocks_; code++) {
        allActions.push_back(std::make_unique<RockSampleAction>(code));
    }
    return allActions;
}

std::vector<std::unique_ptr<solver::DiscretizedPoint>>
RockSampleModel::getAllObservationsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allObservations_;
    for (long code = 0; code < 3; code++) {
        allObservations_.push_back(
                std::make_unique<RockSampleObservation>(code));
    }
    return allObservations_;
}
} /* namespace rocksample */
