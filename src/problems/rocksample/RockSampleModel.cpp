#include "RockSampleModel.hpp"

#include <cmath>                        // for pow, floor
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <fstream>                      // for operator<<, basic_ostream, endl, basic_ostream<>::__ostream_type, ifstream, basic_ostream::operator<<, basic_istream, basic_istream<>::__istream_type
#include <initializer_list>
#include <iostream>                     // for cout
#include <queue>
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

#include "solver/indexing/RTree.hpp"
#include "solver/indexing/FlaggingVisitor.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"
#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/enumerated_observations.hpp"

#include "solver/changes/ChangeFlags.hpp"        // for ChangeFlags

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/StatePool.hpp"

#include "position_history.hpp"
#include "smart_history.hpp"
#include "LegalActionsPool.hpp"
#include "PreferredActionsPool.hpp"
#include "RockSampleAction.hpp"         // for RockSampleAction
#include "RockSampleMdpSolver.hpp"
#include "RockSampleObservation.hpp"    // for RockSampleObservation
#include "RockSampleState.hpp"          // for RockSampleState
#include "RockSampleTextSerializer.hpp"

using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace rocksample {
RockSampleModel::RockSampleModel(RandomGenerator *randGen, po::variables_map vm) :
            ModelWithProgramOptions(randGen, vm),
            goodRockReward_(vm["problem.goodRockReward"].as<double>()),
            badRockPenalty_(vm["problem.badRockPenalty"].as<double>()),
            exitReward_(vm["problem.exitReward"].as<double>()),
            illegalMovePenalty_(vm["problem.illegalMovePenalty"].as<double>()),
            halfEfficiencyDistance_(vm["problem.halfEfficiencyDistance"].as<double>()),
            nRows_(0), // update
            nCols_(0), // update
            nRocks_(0), // update
            startPos_(), // update
            rockPositions_(), // push rocks
            goalPositions_(), // push goals
            mapText_(), // push rows
            envMap_(), // push rows
            goalDistances_(), // calculate distances
            rockDistances_(), // calculate distances
            heuristicType_(parseCategory(vm["heuristics.type"].as<std::string>())),
            searchCategory_(parseCategory(vm["heuristics.search"].as<std::string>())),
            rolloutCategory_(parseCategory(vm["heuristics.rollout"].as<std::string>())),
            usingPreferredInit_(vm["heuristics.usePreferredInit"].as<bool>()),
            preferredQValue_(vm["heuristics.preferredQValue"].as<double>()),
            preferredVisitCount_(vm["heuristics.preferredVisitCount"].as<long>()),
            nStVars_(), // depends on nRocks
            minVal_(-illegalMovePenalty_ / (1 - getDiscountFactor())),
            maxVal_(0), // depends on nRocks
            mdpSolver_(nullptr) {
    if (searchCategory_ > heuristicType_) {
        searchCategory_ = heuristicType_;
    }
    if (rolloutCategory_ > heuristicType_) {
        rolloutCategory_ = heuristicType_;
    }

    registerHeuristicParser("exact", std::make_unique<RockSampleMdpParser>(this));
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
    if (hasVerboseOutput()) {
        cout << "Constructed the RockSampleModel" << endl;
        cout << "Discount: " << getDiscountFactor() << endl;
        cout << "Size: " << nRows_ << " by " << nCols_ << endl;
        cout << "nRocks: " << nRocks_ << endl;

        cout << "Environment:" << endl;
        drawEnv(cout);
        cout << endl;

        cout << "Distances to the goal:" << endl;
        drawDistances(goalDistances_, std::cout);
        cout << endl;

        cout << "Distances to rock #2:" << endl;
        drawDistances(rockDistances_[2], std::cout);
        cout << endl;
    }
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
                cellType = (RSCellType) (ROCK + nRocks_);
                nRocks_++;
            } else if (c == 'G') {
                cellType = GOAL;
                goalPositions_.push_back(p);
            } else if (c == 'S') {
                startPos_ = p;
                cellType = EMPTY;
            } else if (c == 'X') {
                cellType = OBSTACLE;
            } else {
                cellType = EMPTY;
            }
            envMap_.back().push_back(cellType);
        }
    }

    nStVars_ = 2 + nRocks_;
    minVal_ = -illegalMovePenalty_ / (1 - getDiscountFactor());
    maxVal_ = goodRockReward_ * nRocks_ + exitReward_;

    goalDistances_.resize(nRows_);
    for (std::vector<int> &row : goalDistances_) {
        row.resize(nCols_);
    }

    rockDistances_.resize(nRocks_);
    for (auto &distances : rockDistances_) {
        distances.resize(nRows_);
        for (auto &row : distances) {
            row.resize(nCols_);
        }
    }

    recalculateAllDistances();

    if (getDistance(startPos_, -1) == -1){
        cout << "ERROR: Unreachable goal!";
        std::exit(10);
    }
}

void RockSampleModel::recalculateAllDistances() {
    recalculateDistances(goalDistances_, goalPositions_);
    for (int rockNo = 0; rockNo < nRocks_; rockNo++) {
        recalculateDistances(rockDistances_[rockNo],
                std::vector<GridPosition> {rockPositions_[rockNo]});
    }
}

void RockSampleModel::recalculateDistances(std::vector<std::vector<int>> &grid,
        std::vector<GridPosition> targets) {
    // Preinitialize to -1 for all cells.
    for (auto &row : grid) {
        for (auto &cell : row) {
            cell = -1;
        }
    }
    std::queue<GridPosition> queue;
    for (GridPosition &pos : targets) {
        grid[pos.i][pos.j] = 0;
        queue.push(pos);
    }

    while (!queue.empty()) {
        GridPosition pos = queue.front();
        queue.pop();
        int distance = grid[pos.i][pos.j] + 1;
        for (ActionType direction : {ActionType::NORTH, ActionType::SOUTH, ActionType::WEST,
            ActionType::EAST}) {
            GridPosition nextPos;
            bool isLegal;
            std::tie(nextPos, isLegal) = makeNextPosition(pos, direction);
            // The distance matters only if the move is legal, and doesn't move into a goal.
            if (isLegal && getCellType(nextPos) != GOAL) {
                int &nextPosDistance = grid[nextPos.i][nextPos.j];
                if (nextPosDistance == -1 || nextPosDistance > distance) {
                    nextPosDistance = distance;
                    queue.push(nextPos);
                }
            }
        }
    }
}


/* --------------- The model interface proper ----------------- */
std::unique_ptr<solver::State> RockSampleModel::sampleAnInitState() {
    return std::make_unique<RockSampleState>(startPos_, sampleRocks());
//    return std::make_unique<RockSampleState>(startPos_, std::vector<bool> {
//        true, false, true, false, false, false, false, false
//    });
}

std::unique_ptr<solver::State> RockSampleModel::sampleStateUniform() {
    while (true) {
        GridPosition position = samplePosition();
        // States that are on top of an obstacle are completely invalid.
        if (getCellType(position) != OBSTACLE) {
            return std::make_unique<RockSampleState>(position, sampleRocks());
        }
    }
    return nullptr;
}

GridPosition RockSampleModel::samplePosition() {
    long i = std::uniform_int_distribution<long>(0, nRows_ - 1)(*getRandomGenerator());
    long j = std::uniform_int_distribution<long>(0, nCols_ - 1)(*getRandomGenerator());
    return GridPosition(i, j);
}

std::vector<bool> RockSampleModel::sampleRocks() {
    return decodeRocks(
            std::uniform_int_distribution<long>(0, (1 << nRocks_) - 1)(*getRandomGenerator()));
}

std::vector<bool> RockSampleModel::decodeRocks(long val) {
    std::vector<bool> rockStates;
    for (int j = 0; j < nRocks_; j++) {
        rockStates.push_back(val & (1 << j));
    }
    return rockStates;
}

long RockSampleModel::encodeRocks(std::vector<bool> rockStates) {
    long value = 0;
    for (int j = 0; j < nRocks_; j++) {
        if (rockStates[j]) {
            value += (1 << j);
        }
    }
    return value;
}

bool RockSampleModel::isTerminal(solver::State const &state) {
    RockSampleState const &rockSampleState = static_cast<RockSampleState const &>(state);
    GridPosition pos = rockSampleState.getPosition();
    return getCellType(pos) == GOAL;
}

GridPosition RockSampleModel::makeAdjacentPosition(GridPosition position, ActionType actionType) {
    if (actionType == ActionType::NORTH) {
        position.i -= 1;
    } else if (actionType == ActionType::EAST) {
        position.j += 1;
    } else if (actionType == ActionType::SOUTH) {
        position.i += 1;
    } else if (actionType == ActionType::WEST) {
        position.j -= 1;
    }
    return position;
}

std::pair<GridPosition, bool> RockSampleModel::makeNextPosition(GridPosition position,
        ActionType actionType) {

    GridPosition oldPosition = position;

    bool isLegal = true;
    if (actionType == ActionType::CHECK) {
        // Do nothing - the state remains the same.
    } else if (actionType == ActionType::SAMPLE) {
        int rockNo = getCellType(position) - ROCK;
        if (rockNo < 0 || rockNo >= nRocks_) {
            isLegal = false;
        }
    } else if (actionType == ActionType::NORTH) {
        if (position.i > 0) {
            position.i -= 1;
        } else {
            isLegal = false;
        }
    } else if (actionType == ActionType::EAST) {
        if (position.j < nCols_ - 1) {
            position.j += 1;
        } else {
            isLegal = false;
        }
    } else if (actionType == ActionType::SOUTH) {
        if (position.i < nRows_ - 1) {
            position.i += 1;
        } else {
            isLegal = false;
        }
    } else if (actionType == ActionType::WEST) {
        if (position.j > 0) {
            position.j -= 1;
        } else {
            isLegal = false;
        }
    } else {
        std::ostringstream message;
        message << "Invalid action: " << actionType;
        debug::show_message(message.str());
        isLegal = false;
    }

    if (isLegal) {
        // Moving into obstacles is invalid!
        if (getCellType(position) == OBSTACLE) {
            isLegal = false;
            position = oldPosition;
        }

    }
    return std::make_pair(position, isLegal);
}

/* -------------------- Black box dynamics ---------------------- */
std::pair<std::unique_ptr<RockSampleState>, bool> RockSampleModel::makeNextState(
        RockSampleState const &state, RockSampleAction const &action) {

    GridPosition nextPos;
    bool isLegal;
    std::tie(nextPos, isLegal) = makeNextPosition(state.getPosition(), action.getActionType());
    if (!isLegal) {
        return std::make_pair(std::make_unique<RockSampleState>(state), false);
    }

    std::vector<bool> rockStates(state.getRockStates());
    ActionType actionType = action.getActionType();
    if (actionType == ActionType::SAMPLE) {
        int rockNo = getCellType(nextPos) - ROCK;
        rockStates[rockNo] = false;
    }

    return std::make_pair(std::make_unique<RockSampleState>(nextPos, rockStates), true);
}

std::unique_ptr<RockSampleObservation> RockSampleModel::makeObservation(
        RockSampleAction const &action, RockSampleState const &nextState) {
    ActionType actionType = action.getActionType();
    if (actionType < ActionType::CHECK) {
        return std::make_unique<RockSampleObservation>();
    }
    long rockNo = action.getRockNo();
    GridPosition pos(nextState.getPosition());
    std::vector<bool> rockStates(nextState.getRockStates());
    double dist = pos.euclideanDistanceTo(rockPositions_[rockNo]);
    bool obsMatches = std::bernoulli_distribution(getSensorCorrectnessProbability(dist))(
            *getRandomGenerator());
    return std::make_unique<RockSampleObservation>(rockStates[rockNo] == obsMatches);
}

double RockSampleModel::makeReward(RockSampleState const &state, RockSampleAction const &action,
        RockSampleState const &nextState, bool isLegal) {
    if (!isLegal) {
        return -illegalMovePenalty_;
    }
    if (isTerminal(nextState)) {
        return exitReward_;
    }

    ActionType actionType = action.getActionType();
    if (actionType == ActionType::SAMPLE) {
        GridPosition pos = state.getPosition();
        int rockNo = getCellType(pos) - ROCK;
        if (0 <= rockNo && rockNo < nRocks_) {
            return state.getRockStates()[rockNo] ? goodRockReward_ : -badRockPenalty_;
        } else {
            debug::show_message("Invalid sample action!?!");
            return -illegalMovePenalty_;
        }
    }
    return 0;
}

std::unique_ptr<solver::State> RockSampleModel::generateNextState(solver::State const &state,
        solver::Action const &action, solver::TransitionParameters const */*tp*/) {
    return makeNextState(static_cast<RockSampleState const &>(state),
            static_cast<RockSampleAction const &>(action)).first;
}

std::unique_ptr<solver::Observation> RockSampleModel::generateObservation(
        solver::State const */*state*/, solver::Action const &action,
        solver::TransitionParameters const */*tp*/, solver::State const &nextState) {
    return makeObservation(static_cast<RockSampleAction const &>(action),
            static_cast<RockSampleState const &>(nextState));
}

double RockSampleModel::generateReward(solver::State const &state, solver::Action const &action,
        solver::TransitionParameters const */*tp*/, solver::State const */*nextState*/) {
    RockSampleState const &rockSampleState = (static_cast<RockSampleState const &>(state));
    RockSampleAction const &rockSampleAction = (static_cast<RockSampleAction const &>(action));
    std::unique_ptr<RockSampleState> nextState;
    bool isLegal;
    std::tie(nextState, isLegal) = makeNextState(rockSampleState, rockSampleAction);
    return makeReward(rockSampleState, rockSampleAction, *nextState, isLegal);
}

solver::Model::StepResult RockSampleModel::generateStep(solver::State const &state,
        solver::Action const &action) {
    RockSampleState const &rockSampleState = static_cast<RockSampleState const &>(state);
    RockSampleAction const &rockSampleAction = (static_cast<RockSampleAction const &>(action));
    solver::Model::StepResult result;
    result.action = action.copy();

    bool isLegal;
    std::unique_ptr<RockSampleState> nextState;
    std::tie(nextState, isLegal) = makeNextState(rockSampleState, rockSampleAction);
    result.observation = makeObservation(rockSampleAction, *nextState);
    result.reward = makeReward(rockSampleState, rockSampleAction, *nextState, isLegal);
    result.isTerminal = isTerminal(*nextState);
    result.nextState = std::move(nextState);
    return result;
}

/* -------------- Methods for handling model changes ---------------- */
void RockSampleModel::applyChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
            solver::Solver *solver) {
    solver::StatePool *pool = nullptr;
    if (solver != nullptr) {
        pool = solver->getStatePool();
    }

    if (hasVerboseOutput() && pool != nullptr)  {
        cout << "Applying model changes..." << endl;
    }

    solver::Heuristic heuristic = getHeuristicFunction();

    std::vector<double> allHeuristicValues;
    if (pool != nullptr) {
        long nStates = pool->getNumberOfStates();
        allHeuristicValues.resize(nStates);
        for (long index = 0; index < nStates; index++) {
            allHeuristicValues[index] = heuristic(nullptr, pool->getInfoById(index)->getState(),
                    nullptr);
        }
    }

    // The cells that have been affected by these changes.
    std::unordered_set<GridPosition> affectedCells;

    for (auto const &change : changes) {
        RockSampleChange const &rsChange = static_cast<RockSampleChange const &>(*change);
        if (hasVerboseOutput()) {
            cout << rsChange.changeType << " " << rsChange.i0 << " "
                    << rsChange.j0;
            cout << " " << rsChange.i1 << " " << rsChange.j1 << endl;
        }

        RSCellType newCellType;
        if (rsChange.changeType == "Add Obstacles") {
            newCellType = OBSTACLE;
        } else if (rsChange.changeType == "Remove Obstacles") {
            newCellType = EMPTY;
        } else {
            cout << "Invalid change type: " << rsChange.changeType;
            continue;
        }

        for (long i = static_cast<long>(rsChange.i0); i <= rsChange.i1; i++) {
            for (long j = static_cast<long>(rsChange.j0); j <= rsChange.j1; j++) {
                RSCellType oldCellType = envMap_[i][j];
                envMap_[i][j] = newCellType;
                if (newCellType != oldCellType) {
                    affectedCells.insert(GridPosition(i, j));
                }
            }
        }

        if (pool == nullptr) {
            continue;
        }

        if (searchCategory_ == RSActionCategory::LEGAL && newCellType == RSCellType::EMPTY) {
            // Legal actions + newly empty cells => handled automatically.
        }

        // If we're adding obstacles, we need to mark the invalid states as deleted.
        solver::RTree *tree = static_cast<solver::RTree *>(pool->getStateIndex());

        std::vector<double> lowCorner;
        std::vector<double> highCorner;
        for (int i = 0; i < nRocks_+2; i++) {
            lowCorner.push_back(0.0);
            highCorner.push_back(1.0);
        }
        lowCorner[0] = rsChange.i0;
        lowCorner[1] = rsChange.j0;
        highCorner[0] = rsChange.i1;
        highCorner[1] = rsChange.j1;


        solver::ChangeFlags flags = solver::ChangeFlags::DELETED;
        if (newCellType == RSCellType::EMPTY) {
            flags = solver::ChangeFlags::TRANSITION;
            lowCorner[0] -= 1;
            lowCorner[1] -= 1;
            highCorner[0] += 1;
            highCorner[1] += 1;
        }

        solver::FlaggingVisitor visitor(pool, flags);
        tree->boxQuery(visitor, lowCorner, highCorner);
    }

    //
    if (searchCategory_ == RSActionCategory::LEGAL) {
        LegalActionsPool *actionPool = static_cast<LegalActionsPool *>(solver->getActionPool());
        for (GridPosition cell : affectedCells) {
            bool isLegal = (envMap_[cell.i][cell.j] != OBSTACLE);
            for (ActionType actionType : {ActionType::NORTH, ActionType::EAST, ActionType::SOUTH,
                ActionType::WEST}) {
                GridPosition adjacentCell = makeAdjacentPosition(cell, actionType);
                // Ignore cells that are out of bounds.
                if (!isWithinBounds(adjacentCell)) {
                    continue;
                }
                RockSampleAction rsAction(actionType);
                actionPool->setLegal(isLegal, adjacentCell, rsAction);
            }
        }
    }

    // Recalculate all the distances.
    recalculateAllDistances();

    if (mdpSolver_ != nullptr) {
        mdpSolver_->solve();
    }

    // Check for heuristic changes.
    if (pool != nullptr) {
        long nStates = pool->getNumberOfStates();
        for (long index = 0; index < nStates; index++) {
            double oldValue = allHeuristicValues[index];
            solver::StateInfo *info = pool->getInfoById(index);
            double newValue = heuristic(nullptr, info->getState(), nullptr);
            if (std::abs(newValue - oldValue) > 1e-5) {
                pool->setChangeFlags(info, solver::ChangeFlags::HEURISTIC);
            }
        }
    }

    if (hasVerboseOutput() && pool != nullptr) {
        cout << "Done applying model changes..." << endl;
    }
}


/* ------------ Methods for handling particle depletion -------------- */
std::vector<std::unique_ptr<solver::State>> RockSampleModel::generateParticles(
        solver::BeliefNode */*previousBelief*/, solver::Action const &action,
        solver::Observation const &obs, long nParticles,
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
            RockSampleState const *rockSampleState = static_cast<RockSampleState const *>(state);
            GridPosition pos(rockSampleState->getPosition());
            double dist = pos.euclideanDistanceTo(rockPositions_[rockNo]);
            bool rockIsGood = rockSampleState->getRockStates()[rockNo];

            double probability = getSensorCorrectnessProbability(dist);
            RockSampleObservation const &observation =
                    (static_cast<RockSampleObservation const &>(obs));
            if (rockIsGood != observation.isGood()) {
                probability = 1 - probability;
            }
            weights[*rockSampleState] += probability;
            weightTotal += probability;
        }
        double scale = nParticles / weightTotal;
        for (WeightMap::value_type &it : weights) {
            double proportion = it.second * scale;
            long numToAdd = static_cast<long>(proportion);
            if (std::bernoulli_distribution(proportion - numToAdd)(*getRandomGenerator())) {
                numToAdd += 1;
            }
            for (int i = 0; i < numToAdd; i++) {
                newParticles.push_back(std::make_unique<RockSampleState>(it.first));
            }
        }

    } else {
        // It's not a CHECK action, so we just add each resultant state.
        for (solver::State const *state : previousParticles) {
            RockSampleState const *rockSampleState = (static_cast<RockSampleState const *>(state));
            RockSampleAction const &rockSampleAction =
                    (static_cast<RockSampleAction const &>(action));
            newParticles.push_back(makeNextState(*rockSampleState, rockSampleAction).first);
        }
    }
    return newParticles;
}

std::vector<std::unique_ptr<solver::State>> RockSampleModel::generateParticles(
        solver::BeliefNode */*previousBelief*/, solver::Action const &action,
        solver::Observation const &obs, long nParticles) {
    std::vector<std::unique_ptr<solver::State>> particles;
    while ((long) particles.size() < nParticles) {
        std::unique_ptr<solver::State> state = sampleStateUniform();
        solver::Model::StepResult result = generateStep(*state, action);
        if (obs == *result.observation) {
            particles.push_back(std::move(result.nextState));
        }
    }
    return particles;
}

/* ------------------- Pretty printing methods --------------------- */
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
    case OBSTACLE:
        os << 'X';
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

void RockSampleModel::drawDistances(std::vector<std::vector<int>> &grid, std::ostream &os) {
    for (auto &row : grid) {
        for (int cellValue : row) {
            if (cellValue == -1) {
                os << ".";
            } else {
                os << std::hex << cellValue;
            }
        }
        os << endl;
    }
}

void RockSampleModel::drawSimulationState(solver::BeliefNode const *belief,
        solver::State const &state, std::ostream &os) {
    RockSampleState const &rockSampleState = static_cast<RockSampleState const &>(state);
    std::vector<solver::State const *> particles = belief->getStates();
    GridPosition pos(rockSampleState.getPosition());
    std::vector<double> goodProportions(nRocks_);
    for (solver::State const *particle : particles) {
        RockSampleState const &rss = static_cast<RockSampleState const &>(*particle);
        for (long i = 0; i < nRocks_; i++) {
            if (rss.getRockStates()[i]) {
                goodProportions[i] += 1;
            }
        }
    }
    for (long i = 0; i < nRocks_; i++) {
        goodProportions[i] /= particles.size();
    }

    std::vector<int> colors { 196, 161, 126, 91, 56, 21, 26, 31, 36, 41, 46 };
    if (hasColorOutput()) {
        os << "Color map: ";
        for (int color : colors) {
            os << "\033[38;5;" << color << "m";
            os << '*';
            os << "\033[0m";
        }
        os << endl;
    }
    for (std::size_t i = 0; i < envMap_.size(); i++) {
        for (std::size_t j = 0; j < envMap_[0].size(); j++) {
            long rockNo = envMap_[i][j] - ROCK;
            if (rockNo >= 0 && hasColorOutput()) {
                int color = colors[goodProportions[rockNo] * (colors.size() - 1)];
                os << "\033[38;5;" << color << "m";
            }
            if ((long) i == pos.i && (long) j == pos.j) {
                os << "r";
            } else {
                dispCell(envMap_[i][j], os);
            }
            if (rockNo >= 0 && hasColorOutput()) {
                os << "\033[0m";
            }
        }
        os << endl;
    }
    for (double p : goodProportions) {
        os << p << " ";
    }
    os << endl;
}

/* ---------------------- Basic customizations  ---------------------- */
double RockSampleModel::getDefaultHeuristicValue(solver::HistoryEntry const */*entry*/,
        solver::State const *state, solver::HistoricalData const */*data*/) {
    RockSampleState const &rockSampleState = static_cast<RockSampleState const &>(*state);
    double qVal = 0;
    double currentDiscount = 1;
    GridPosition currentPos(rockSampleState.getPosition());
    std::vector<bool> rockStates(rockSampleState.getRockStates());

    std::set<int> goodRocks;
    for (int i = 0; i < nRocks_; i++) {
        // Only bother with reachable rocks.
        if (rockStates[i] && getDistance(currentPos, i) != -1) {
            goodRocks.insert(i);
        }
    }

    // Visit the rocks in a greedy order.
    while (!goodRocks.empty()) {
        std::set<int>::iterator it = goodRocks.begin();
        int bestRock = *it;
        long lowestDist = getDistance(currentPos, bestRock);
        ++it;
        for (; it != goodRocks.end(); ++it) {
            long dist = getDistance(currentPos, *it);
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

    // Now move to a goal square.
    currentDiscount *= std::pow(getDiscountFactor(), getDistance(currentPos, -1));
    qVal += currentDiscount * exitReward_;
    return qVal;
}

std::unique_ptr<RockSampleAction> RockSampleModel::getRandomAction() {
    long binNumber = std::uniform_int_distribution<int>(0, 4 + nRocks_)(*getRandomGenerator());
    return std::make_unique<RockSampleAction>(binNumber);
}
std::unique_ptr<RockSampleAction> RockSampleModel::getRandomAction(std::vector<long> binNumbers) {
    if (binNumbers.empty()) {
        return nullptr;
    }
    long index = std::uniform_int_distribution<int>(0, binNumbers.size() - 1)(
            *getRandomGenerator());
    return std::make_unique<RockSampleAction>(binNumbers[index]);
}

std::unique_ptr<solver::Action> RockSampleModel::getRolloutAction(
        solver::HistoricalData const *data, solver::State const */*state*/) {
    if (rolloutCategory_ == RSActionCategory::ALL) {
        return getRandomAction();
    } else if (rolloutCategory_ == RSActionCategory::LEGAL) {
        if (heuristicType_ == RSActionCategory::LEGAL) {
            return getRandomAction(static_cast<PositionData const &>(*data).generateLegalActions());
        } else if (heuristicType_ == RSActionCategory::PREFERRED) {
            return getRandomAction(
                    static_cast<PositionAndRockData const &>(*data).generateLegalActions());
        }
    } else {
        return getRandomAction(
                static_cast<PositionAndRockData const &>(*data).generatePreferredActions());
    }
    return nullptr;
}

/* ------- Customization of more complex solver functionality  --------- */
std::vector<std::unique_ptr<solver::DiscretizedPoint>> RockSampleModel::getAllActionsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allActions;
    for (long code = 0; code < 5 + nRocks_; code++) {
        allActions.push_back(std::make_unique<RockSampleAction>(code));
    }
    return std::move(allActions);
}

std::unique_ptr<solver::ActionPool> RockSampleModel::createActionPool(solver::Solver */*solver*/) {
    switch (heuristicType_) {
    case RSActionCategory::LEGAL:
        return std::make_unique<LegalActionsPool>(this);
    case RSActionCategory::PREFERRED:
        return std::make_unique<PreferredActionsPool>(this);
    default:
        return std::make_unique<solver::EnumeratedActionPool>(this, getAllActionsInOrder());
    }
}
std::unique_ptr<solver::HistoricalData> RockSampleModel::createRootHistoricalData() {
    switch (heuristicType_) {
    case RSActionCategory::LEGAL:
        return std::make_unique<PositionData>(this, getStartPosition());
    case RSActionCategory::PREFERRED:
        return std::make_unique<PositionAndRockData>(this, getStartPosition());
    default:
        return nullptr;
    }
}

std::vector<std::unique_ptr<solver::DiscretizedPoint>> RockSampleModel::getAllObservationsInOrder() {
    std::vector<std::unique_ptr<solver::DiscretizedPoint>> allObservations_;
    for (long code = 0; code < 3; code++) {
        allObservations_.push_back(std::make_unique<RockSampleObservation>(code));
    }
    return allObservations_;
}
std::unique_ptr<solver::ObservationPool> RockSampleModel::createObservationPool(
        solver::Solver */*solver*/) {
    return std::make_unique<solver::EnumeratedObservationPool>(getAllObservationsInOrder());
}

std::unique_ptr<solver::Serializer> RockSampleModel::createSerializer(solver::Solver *solver) {
    switch (heuristicType_) {
    case RSActionCategory::LEGAL:
        return std::make_unique<RockSampleLegalActionsTextSerializer>(solver);
    case RSActionCategory::PREFERRED:
        return std::make_unique<RockSamplePreferredActionsTextSerializer>(solver);
    default:
        return std::make_unique<RockSampleTextSerializer>(solver);
    }
}
} /* namespace rocksample */
