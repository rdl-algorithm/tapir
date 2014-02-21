#include "Nav2DModel.hpp"

#include <cmath>                        // for floor, pow
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <memory>
#include <fstream>                      // for ifstream, basic_istream, basic_istream<>::__istream_type
#include <iomanip>                      // for operator<<, setw
#include <iostream>                     // for cout, cerr
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <unordered_map>                // for _Node_iterator, operator!=, unordered_map<>::iterator, _Node_iterator_base, unordered_map
#include <utility>                      // for make_pair, move, pair

#include <boost/program_options.hpp>    // for variables_map, variable_value

#include "global.hpp"                     // for RandomGenerator, make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator!=, operator<<
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions
#include "solver/Action.hpp"            // for Action
#include "solver/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/FlaggingVisitor.hpp"
#include "solver/Model.hpp"             // for Model::StepResult, Model
#include "solver/Observation.hpp"       // for Observation
#include "solver/RTree.hpp"
#include "solver/SpatialIndexVisitor.hpp"             // for State, State::Hash, operator<<, operator==
#include "solver/VectorLP.hpp"             // for State, State::Hash, operator<<, operator==
#include "solver/State.hpp"             // for State, State::Hash, operator<<, operator==
#include "solver/StatePool.hpp"


#include "Nav2DState.hpp"                 // for Nav2DState

using std::cerr;
using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace nav2d {
Nav2DModel::Nav2DModel(RandomGenerator *randGen, po::variables_map vm) :
    ModelWithProgramOptions(randGen, vm),
    moveCost_(vm["problem.moveCost"].as<double>()),
    nav2dReward_(vm["problem.nav2dReward"].as<double>()),
    failedNav2DPenalty_(vm["problem.failedNav2DPenalty"].as<double>()),
    opponentStayProbability_(
            vm["problem.opponentStayProbability"].as<double>()),
    nRows_(0), // to be updated
    nCols_(0), // to be updated
    nEmptyCells_(0), // will count
    mapText_(), // will be pushed to
    envMap_(), // will be pushed to
    changes_(),

    nActions_(5),
    nObservations_(0), // to be updated
    nStVars_(5),
    minVal_(-failedNav2DPenalty_ / (1 - getDiscountFactor())),
    maxVal_(nav2dReward_)
    {
    // Read the map from the file.
    std::ifstream inFile;
    char const *mapPath = vm["problem.mapPath"].as<std::string>().c_str();
    inFile.open(mapPath);
    if (!inFile.is_open()) {
        std::cerr << "Failed to open " << mapPath << "\n";
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
    cout << "Constructed the Nav2DModel" << endl;
    cout << "Discount: " << getDiscountFactor() << endl;
    cout << "Size: " << nRows_ << " by " << nCols_ << endl;
    cout << "move cost: " << moveCost_ << endl;
    cout << "nActions: " << nActions_ << endl;
    cout << "nObservations: " << nObservations_ << endl;
    cout << "nStVars: " << nStVars_ << endl;
    cout << "Example States: " << endl;
    for (int i = 0; i < 5; i++) {
        std::unique_ptr<solver::State> state = sampleAnInitState();
        cout << *state << " Heuristic: " << getHeuristicValue(*state) << endl;
    }
    cout << "nParticles: " << getNParticles() << endl;
    cout << "Environment:" << endl << endl;
    drawEnv(cout);
}

void Nav2DModel::initialize() {
    GridPosition p;
    nEmptyCells_ = 0;
    envMap_.resize(nRows_);
    for (p.i = nRows_ - 1; p.i >= 0; p.i--) {
        envMap_[p.i].resize(nCols_);
        for (p.j = 0; p.j < nCols_; p.j++) {
            char c = mapText_[p.i][p.j];
            Nav2DCellType cellType;
            if (c == 'X') {
                cellType = WALL;
            } else {
                cellType = Nav2DCellType::EMPTY;
                nEmptyCells_++;
            }
            envMap_[p.i][p.j] = cellType;
        }
    }

    nObservations_ = nEmptyCells_ * 2;
}

GridPosition Nav2DModel::randomEmptyCell() {
    GridPosition pos;
    while (true) {
        pos.i = std::uniform_int_distribution<long>(0, nRows_ - 1)(*randGen_);
        pos.j = std::uniform_int_distribution<long>(0, nCols_ - 1)(*randGen_);
        if (envMap_[pos.i][pos.j] == Nav2DCellType::EMPTY) {
            break;
        }
    }
    return pos;
}

std::unique_ptr<solver::State> Nav2DModel::sampleAnInitState() {
    return sampleStateUniform();
}

std::unique_ptr<solver::State> Nav2DModel::sampleStateUniform() {
    GridPosition robotPos = randomEmptyCell();
    GridPosition opponentPos = randomEmptyCell();
    return std::make_unique<Nav2DState>(robotPos, opponentPos, false);
}

bool Nav2DModel::isTerminal(solver::State const &state) {
    return static_cast<Nav2DState const &>(state).isNav2Dged();
}

double Nav2DModel::getHeuristicValue(solver::State const &state) {
    Nav2DState const &nav2dState = static_cast<Nav2DState const &>(state);
    if (nav2dState.isNav2Dged()) {
        return 0;
    }
    GridPosition robotPos = nav2dState.getRobotPosition();
    GridPosition opponentPos = nav2dState.getOpponentPosition();
    int dist = robotPos.manhattanDistanceTo(opponentPos);
    double nSteps = dist / opponentStayProbability_;
    double finalDiscount = std::pow(getDiscountFactor(), nSteps);
    double qVal = -moveCost_ * (1 - finalDiscount) / (1 - getDiscountFactor());
    qVal += finalDiscount * nav2dReward_;
    return qVal;
}

double Nav2DModel::getDefaultVal() {
    return minVal_;
}

std::pair<std::unique_ptr<Nav2DState>, bool> Nav2DModel::makeNextState(
        solver::State const &state, solver::Action const &action) {
    Nav2DState const &nav2dState = static_cast<Nav2DState const &>(state);
    if (nav2dState.isNav2Dged()) {
        return std::make_pair(std::make_unique<Nav2DState>(nav2dState), false);
    }

    GridPosition robotPos = nav2dState.getRobotPosition();
    GridPosition opponentPos = nav2dState.getOpponentPosition();
    if (action == Nav2DAction::NAV2D && robotPos == opponentPos) {
        return std::make_pair(
                std::make_unique<Nav2DState>(robotPos, opponentPos, true), true);
    }

    GridPosition newOpponentPos = getMovedOpponentPos(robotPos, opponentPos);
    GridPosition newRobotPos = getMovedPos(robotPos, action);
    if (!isValid(newRobotPos)) {
        return std::make_pair(
                std::make_unique<Nav2DState>(robotPos, newOpponentPos, false),
                false);
    }
    return std::make_pair(std::make_unique<Nav2DState>(
                    newRobotPos, newOpponentPos, false), true);
}

std::vector<Nav2DModel::Nav2DAction> Nav2DModel::makeOpponentActions(
        GridPosition const &robotPos,
        GridPosition const &opponentPos) {
    std::vector<Nav2DAction> actions;
    if (robotPos.i > opponentPos.i) {
        actions.push_back(NORTH);
        actions.push_back(NORTH);
    } else if (robotPos.i < opponentPos.i) {
        actions.push_back(SOUTH);
        actions.push_back(SOUTH);
    } else {
        actions.push_back(NORTH);
        actions.push_back(SOUTH);
    }
    if (robotPos.j > opponentPos.j) {
        actions.push_back(WEST);
        actions.push_back(WEST);
    } else if (robotPos.j < opponentPos.j) {
        actions.push_back(EAST);
        actions.push_back(EAST);
    } else {
        actions.push_back(EAST);
        actions.push_back(WEST);
    }
    return actions;
}

GridPosition Nav2DModel::getMovedOpponentPos(GridPosition const &robotPos,
        GridPosition const &opponentPos) {
    // Randomize to see if the opponent stays still.
    if (std::bernoulli_distribution(opponentStayProbability_)(*randGen_)) {
        return opponentPos;
    }
    std::vector<Nav2DAction> actions(makeOpponentActions(robotPos, opponentPos));;
    solver::Action action =
        actions[std::uniform_int_distribution<long>(
                    0, actions.size() - 1)(*randGen_)];
    GridPosition newOpponentPos = getMovedPos(opponentPos, action);
    if (!isValid(newOpponentPos)) {
        newOpponentPos = opponentPos;
    }
    return newOpponentPos;
}

GridPosition Nav2DModel::getMovedPos(GridPosition const &position,
        solver::Action const &action) {
    GridPosition movedPos = position;
    switch (action) {
    case NORTH:
        movedPos.i -= 1;
        break;
    case EAST:
        movedPos.j += 1;
        break;
    case SOUTH:
        movedPos.i += 1;
        break;
    case WEST:
        movedPos.j -= 1;
        break;
    case NAV2D:
        break;
    default:
        cerr << "Invalid action: " << action << endl;
        break;
    }
    return movedPos;
}

bool Nav2DModel::isValid(GridPosition const &position) {
    return (position.i >= 0 && position.i < nRows_ && position.j >= 0
            && position.j < nCols_ && envMap_[position.i][position.j] != WALL);
}

std::unique_ptr<solver::Observation> Nav2DModel::makeObservation(
        solver::Action const & /*action*/,
        Nav2DState const &state) {
    std::vector<double> obs(3);
    GridPosition p = state.getRobotPosition();
    obs[0] = p.i;
    obs[1] = p.j;
    if (p == state.getOpponentPosition()) {
        obs[2] = SEEN;
    } else {
        obs[2] = UNSEEN;
    }
    return std::make_unique<solver::VectorLP>(1, obs);
}

double Nav2DModel::getReward(solver::State const &state,
        solver::Action const &action) {
    if (action == NAV2D) {
        Nav2DState const &nav2dState = static_cast<Nav2DState const &>(state);
        if (nav2dState.getRobotPosition() == nav2dState.getOpponentPosition()) {
            return nav2dReward_;
        } else {
            return -failedNav2DPenalty_;
        }
    } else {
        return -moveCost_;
    }
}

std::unique_ptr<solver::State> Nav2DModel::generateNextState(
           solver::State const &state, solver::Action const &action) {
    return makeNextState(static_cast<Nav2DState const &>(state), action).first;
}

std::unique_ptr<solver::Observation> Nav2DModel::generateObservation(
           solver::Action const &action, solver::State const &nextState) {
    return makeObservation(action, static_cast<Nav2DState const &>(nextState));
}

solver::Model::StepResult Nav2DModel::generateStep(solver::State const &state,
        solver::Action const &action) {
    solver::Model::StepResult result;
    result.action = action;
    std::unique_ptr<Nav2DState> nextState = makeNextState(state, action).first;

    result.observation = makeObservation(action, *nextState);
    result.immediateReward = getReward(state, action);
    result.isTerminal = isTerminal(*nextState);
    result.nextState = std::move(nextState);
    return result;
}

std::vector<std::unique_ptr<solver::State>> Nav2DModel::generateParticles(
        solver::Action const &action, solver::Observation const &obs,
        std::vector<solver::State const *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;

    struct Hash {
        std::size_t operator()(Nav2DState const &state) const {
            return state.hash();
        }
    };
    typedef std::unordered_map<Nav2DState, double, Hash> WeightMap;
    WeightMap weights;
    double weightTotal = 0;
    GridPosition newRobotPos(obsVals[0], obsVals[1]);
    if (obsVals[2] == SEEN) {
        // If we saw the opponent, we must be in the same place.
        newParticles.push_back(std::make_unique<Nav2DState>(newRobotPos,
                        newRobotPos, action == NAV2D));
    } else {
        // We didn't see the opponent, so we must be in different places.
        for (solver::State const *state : previousParticles) {
            Nav2DState const *nav2dState = static_cast<Nav2DState const *>(state);
            GridPosition oldRobotPos(nav2dState->getRobotPosition());
            // Ignore states that do not match knowledge of the robot's position.
            if (newRobotPos != getMovedPos(oldRobotPos, action)) {
                continue;
            }
            GridPosition oldOpponentPos(nav2dState->getOpponentPosition());
            std::vector<Nav2DAction> actions(makeOpponentActions(oldRobotPos,
                            oldOpponentPos));
            std::vector<Nav2DAction> newActions;
            for (Nav2DAction enemyAction : actions) {
                if (getMovedPos(oldOpponentPos, enemyAction) != newRobotPos) {
                    newActions.push_back(enemyAction);
                }
            }
            double probability = 1.0 / newActions.size();
            for (solver::Action enemyAction : newActions) {
                GridPosition newOpponentPos = getMovedPos(oldOpponentPos,
                            enemyAction);
                Nav2DState newState(newRobotPos, newOpponentPos, false);
                weights[newState] += probability;
                weightTotal += probability;
            }
        }
        double scale = getNParticles() / weightTotal;
        for (WeightMap::iterator it = weights.begin(); it != weights.end();
             it++) {
            double proportion = it->second * scale;
            int numToAdd = std::floor(proportion);
            if (std::bernoulli_distribution(proportion-numToAdd)(*randGen_)) {
                numToAdd += 1;
            }
            for (int i = 0; i < numToAdd; i++) {
                newParticles.push_back(std::make_unique<Nav2DState>(it->first));
            }
        }
    }
    return newParticles;
}

std::vector<std::unique_ptr<solver::State>> Nav2DModel::generateParticles(
        solver::Action const &action, solver::Observation const &obs) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    GridPosition newRobotPos(obsVals[0], obsVals[1]);
    if (obsVals[2] == SEEN) {
        // If we saw the opponent, we must be in the same place.
        newParticles.push_back(
                std::make_unique<Nav2DState>(newRobotPos, newRobotPos,
                        action == NAV2D));
    } else {
        while (newParticles.size() < getNParticles()) {
            std::unique_ptr<solver::State> state = sampleStateUniform();
            solver::Model::StepResult result = generateStep(*state, action);
            if (obs == *result.observation) {
                newParticles.push_back(std::move(result.nextState));
            }
        }
    }
    return newParticles;
}

std::vector<long> getVector(std::istringstream &iss) {
    std::vector<long> values;
    std::string tmpStr;
    std::getline(iss, tmpStr, '(');
    std::getline(iss, tmpStr, ')');
    std::stringstream iss2(tmpStr);
    while (std::getline(iss2, tmpStr, ',')) {
        values.push_back(std::atoi(tmpStr.c_str()));
    }
    return values;
}

std::vector<long> Nav2DModel::loadChanges(char const *changeFilename) {
    std::vector<long> changeTimes;
    std::ifstream ifs;
    ifs.open(changeFilename);
    std::string line;
    while (std::getline(ifs, line)) {
        std::istringstream sstr(line);
        std::string tmpStr;
        long time;
        long nChanges;
        sstr >> tmpStr >> time >> tmpStr >> nChanges;

        changes_[time] = std::vector<Nav2DChange>();
        changeTimes.push_back(time);
        for (int i = 0; i < nChanges; i++) {
            std::getline(ifs, line);
            sstr.clear();
            sstr.str(line);

            Nav2DChange change;
            std::getline(sstr, change.changeType, ':');
            std::vector<long> v0 = getVector(sstr);
            std::vector<long> v1 = getVector(sstr);
            change.i0 = v0[0];
            change.j0 = v0[1];
            change.i1 = v1[0];
            change.j1 = v1[1];
            changes_[time].push_back(change);
        }
    }
    ifs.close();
    return changeTimes;
}

void Nav2DModel::update(long time, solver::StatePool *pool) {
    for (Nav2DChange &change : changes_[time]) {
        cout << change.changeType << " " << change.i0 << " " << change.j0 << " "
                << change.i1 << " " << change.j1 << endl;
        if (change.changeType == "Add Obstacles") {
            for (int i = change.i0; i <= change.i1; i++) {
                for (int j = change.j0; j <= change.j1; j++) {
                    envMap_[i][j] = Nav2DCellType::WALL;
                }
            }
            solver::RTree *tree =
                    static_cast<solver::RTree *>(pool->getStateIndex());
            solver::FlaggingVisitor visitor(pool,
                    solver::ChangeFlags::DELETED);
            tree->boxQuery(visitor,
                    std::vector<double> { change.i0, change.j0, 0,       0,               0 },
                    std::vector<double> { change.i1, change.j1, nRows_-1.0,  nCols_-1.0,  1 });
            tree->boxQuery(visitor,
                    std::vector<double> { 0,           0,           change.i0, change.j0, 0 },
                    std::vector<double> { nRows_-1.0,  nCols_-1.0,  change.i1, change.j1, 1 });
        } else if (change.changeType == "Remove Obstacles") {
            for (int i = change.i0; i <= change.i1; i++) {
                for (int j = change.j0; j <= change.j1; j++) {
                    envMap_[i][j] = Nav2DCellType::EMPTY;
                }
            }
            solver::RTree *tree =
                    static_cast<solver::RTree *>(pool->getStateIndex());
            solver::FlaggingVisitor visitor(pool, solver::ChangeFlags::TRANSITION);
            tree->boxQuery(visitor,
                    std::vector<double> { change.i0-1, change.j0-1, 0,             0,             0 },
                    std::vector<double> { change.i1+1, change.j1+1, nRows_-1.0,    nCols_-1.0,    1 });
            tree->boxQuery(visitor,
                    std::vector<double> { 0,             0,             change.i0-1, change.j0-1, 0 },
                    std::vector<double> { nRows_-1.0,    nCols_-1.0,    change.i1+1, change.j1+1, 1 });
        }
    }
}

void Nav2DModel::dispAct(solver::Action const &action, std::ostream &os) {
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
    case NAV2D:
        os << "NAV2D";
        break;
    default:
        os << "INVALID" << action;
        break;
    }
}

void Nav2DModel::dispObs(solver::Observation const &obs, std::ostream &os) {
    os << GridPosition(obsVals[0], obsVals[1]);
    if (obsVals[2] == SEEN) {
        os << " SEEN!";
    }
}

void Nav2DModel::dispCell(Nav2DCellType cellType, std::ostream &os) {
    if (cellType >= EMPTY) {
        os << std::setw(2);
        os << cellType;
        return;
    }
    switch (cellType) {
    case WALL:
        os << "XX";
        break;
    default:
        os << "ERROR-" << cellType;
        break;
    }
}

void Nav2DModel::drawEnv(std::ostream &os) {
    for (std::vector<Nav2DCellType> &row : envMap_) {
        for (Nav2DCellType cellType : row) {
            dispCell(cellType, os);
            os << " ";
        }
        os << endl;
    }
}

void Nav2DModel::drawState(solver::State const &state, std::ostream &os) {
    Nav2DState const &nav2dState = static_cast<Nav2DState const &>(state);
    for (std::size_t i = 0; i < envMap_.size(); i++) {
        for (std::size_t j = 0; j < envMap_[0].size(); j++) {
            GridPosition pos(i, j);
            bool hasRobot = (pos == nav2dState.getRobotPosition());
            bool hasOpponent = (pos == nav2dState.getOpponentPosition());
            if (hasRobot) {
                if (hasOpponent) {
                    os << "#";
                } else {
                    os << "r";
                }
            } else if (hasOpponent) {
                os << "o";
            } else {
                if (envMap_[i][j] == WALL) {
                    os << "X";
                } else {
                    os << ".";
                }
            }
        }
        os << endl;
    }
}
} /* namespace nav2d */
