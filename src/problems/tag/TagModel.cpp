#include "TagModel.hpp"

#include <cmath>                        // for floor, pow
#include <cstddef>                      // for size_t
#include <cstdlib>                      // for exit

#include <fstream>                      // for ifstream, basic_istream, basic_istream<>::__istream_type
#include <iomanip>                      // for operator<<, setw
#include <iostream>                     // for cout, cerr
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <unordered_map>                // for _Node_iterator, operator!=, unordered_map<>::iterator, _Node_iterator_base, unordered_map
#include <utility>                      // for make_pair, move, pair

#include <boost/program_options.hpp>    // for variables_map, variable_value

#include "defs.hpp"                     // for RandomGenerator, make_unique
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator!=, operator<<
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions
#include "solver/Action.hpp"            // for Action
#include "solver/ChangeType.hpp"        // for ChangeType
#include "solver/Model.hpp"             // for Model::StepResult, Model
#include "solver/Observation.hpp"       // for Observation
#include "solver/State.hpp"             // for State, State::Hash, operator<<, operator==

#include "TagState.hpp"                 // for TagState

using std::cerr;
using std::cout;
using std::endl;
namespace po = boost::program_options;

namespace tag {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
TagModel::TagModel(RandomGenerator *randGen, po::variables_map vm) :
    ModelWithProgramOptions(randGen, vm),
    moveCost_(vm["problem.moveCost"].as<double>()),
    tagReward_(vm["problem.tagReward"].as<double>()),
    failedTagPenalty_(vm["problem.failedTagPenalty"].as<double>()),
    opponentStayProbability_(
            vm["problem.opponentStayProbability"].as<double>()) {
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
    cout << "Constructed the TagModel" << endl;
    cout << "Discount: " << getDiscountFactor() << endl;
    cout << "Size: " << nRows_ << " by " << nCols_ << endl;
    cout << "move cost: " << moveCost_ << endl;
    cout << "nActions: " << nActions_ << endl;
    cout << "nObservations: " << nObservations_ << endl;
    cout << "nStVars: " << nStVars_ << endl;
    cout << "Example States: " << endl;
    for (int i = 0; i < 5; i++) {
        std::unique_ptr<solver::State> state = sampleAnInitState();
        cout << *state << " Heuristic: " << solveHeuristic(*state) << endl;
    }
    cout << "nParticles: " << getNParticles() << endl;
    cout << "Environment:" << endl;
    drawEnv(cout);
}

void TagModel::initialise() {
    GridPosition p;
    nEmptyCells_ = 0;
    envMap_.resize(nRows_);
    for (p.i = nRows_ - 1; p.i >= 0; p.i--) {
        envMap_[p.i].resize(nCols_);
        for (p.j = 0; p.j < nCols_; p.j++) {
            char c = mapText_[p.i][p.j];
            TagCellType cellType;
            if (c == 'X') {
                cellType = WALL;
            } else {
                cellType = (TagCellType)(EMPTY + nEmptyCells_);
                emptyCells_.push_back(p);
                nEmptyCells_++;
            }
            envMap_[p.i][p.j] = cellType;
        }
    }

    nActions_ = 5;
    nObservations_ = nEmptyCells_ * 2;
    nStVars_ = 3;
    minVal_ = -failedTagPenalty_ / (1 - getDiscountFactor());
    maxVal_ = tagReward_;
}

long TagModel::encodeGridPosition(GridPosition pos) {
    return envMap_[pos.i][pos.j];
}

GridPosition TagModel::decodeGridPosition(long code) {
    return emptyCells_[code];
}

std::unique_ptr<solver::State> TagModel::sampleAnInitState() {
    return sampleStateUniform();
}

std::unique_ptr<solver::State> TagModel::sampleStateUniform() {
    std::uniform_int_distribution<unsigned long> randomCell(0, nEmptyCells_
            - 1);
    GridPosition robotPos = decodeGridPosition(randomCell(*randGen_));
    GridPosition opponentPos = decodeGridPosition(randomCell(*randGen_));
    return std::make_unique<TagState>(robotPos, opponentPos, false);
}

bool TagModel::isTerm(solver::State const &state) {
    return static_cast<TagState const *>(&state)->isTagged();
}

double TagModel::solveHeuristic(solver::State const &state) {
    TagState const *tagState = static_cast<TagState const *>(&state);
    if (tagState->isTagged()) {
        return 0;
    }
    GridPosition robotPos = tagState->getRobotPosition();
    GridPosition opponentPos = tagState->getOpponentPosition();
    int dist = robotPos.manhattanDistanceTo(opponentPos);
    double nSteps = dist / opponentStayProbability_;
    double finalDiscount = std::pow(getDiscountFactor(), nSteps);
    double qVal = -moveCost_ * (1 - finalDiscount) / (1 - getDiscountFactor());
    qVal += finalDiscount * tagReward_;
    return qVal;
}

double TagModel::getDefaultVal() {
    return minVal_;
}

std::pair<std::unique_ptr<TagState>, bool> TagModel::makeNextState(
        solver::State const &state, solver::Action const &action) {
    TagState const *tagState = static_cast<TagState const *>(&state);
    if (tagState->isTagged()) {
        return std::make_pair(std::make_unique<TagState>(*tagState), false);
    }

    GridPosition robotPos = tagState->getRobotPosition();
    GridPosition opponentPos = tagState->getOpponentPosition();
    if (action == TagAction::TAG && robotPos == opponentPos) {
        return std::make_pair(
                std::make_unique<TagState>(robotPos, opponentPos, true), true);
    }

    GridPosition newOpponentPos = getMovedOpponentPos(robotPos, opponentPos);
    GridPosition newRobotPos = getMovedPos(robotPos, action);
    if (!isValid(newRobotPos)) {
        return std::make_pair(
                std::make_unique<TagState>(robotPos, newOpponentPos, false),
                false);
    }
    return std::make_pair(std::make_unique<TagState>(
                    newRobotPos, newOpponentPos, false), true);
}

std::vector<TagModel::TagAction> TagModel::makeOpponentActions(
        GridPosition const &robotPos,
        GridPosition const &opponentPos) {
    std::vector<TagAction> actions;
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

GridPosition TagModel::getMovedOpponentPos(GridPosition const &robotPos,
        GridPosition const &opponentPos) {
    // Randomize to see if the opponent stays still.
    if (std::bernoulli_distribution(opponentStayProbability_)(*randGen_)) {
        return opponentPos;
    }
    std::vector<TagAction> actions(makeOpponentActions(robotPos, opponentPos));;
    solver::Action action =
        actions[std::uniform_int_distribution<unsigned long>(
                    0, actions.size() - 1)(*randGen_)];
    GridPosition newOpponentPos = getMovedPos(opponentPos, action);
    if (!isValid(newOpponentPos)) {
        newOpponentPos = opponentPos;
    }
    return newOpponentPos;
}

GridPosition TagModel::getMovedPos(GridPosition const &position,
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
    case TAG:
        break;
    default:
        cerr << "Invalid action: " << action;
        break;
    }
    return movedPos;
}

bool TagModel::isValid(GridPosition const &position) {
    return (position.i >= 0 && position.i < nRows_ && position.j >= 0
            && position.j < nCols_ && envMap_[position.i][position.j] != WALL);
}

solver::Observation TagModel::makeObs(solver::Action const & /*action*/,
        TagState const &state) {
    solver::Observation obs(2);
    obs[0] = (double)encodeGridPosition(state.getRobotPosition());
    if (state.getRobotPosition() == state.getOpponentPosition()) {
        obs[1] = SEEN;
    } else {
        obs[1] = UNSEEN;
    }
    return obs;
}

solver::Model::StepResult TagModel::generateStep(solver::State const &state,
        solver::Action const &action) {
    solver::Model::StepResult result;
    result.action = action;
    std::unique_ptr<TagState> nextState = makeNextState(state, action).first;

    result.observation = makeObs(action, *nextState);
    result.immediateReward = getReward(state, action);
    result.isTerminal = isTerm(*nextState);
    result.nextState = std::move(nextState);
    return result;
}

double TagModel::getReward(solver::State const & /*state*/) {
    return 0;
}

double TagModel::getReward(solver::State const &state,
        solver::Action const &action) {
    if (action == TAG) {
        TagState const *tagState = static_cast<TagState const *>(&state);
        if (tagState->getRobotPosition() == tagState->getOpponentPosition()) {
            return tagReward_;
        } else {
            return -failedTagPenalty_;
        }
    } else {
        return -moveCost_;
    }
}


std::vector<std::unique_ptr<solver::State>> TagModel::generateParticles(
        solver::Action const &action, solver::Observation const &obs,
        std::vector<solver::State *> const &previousParticles) {
    std::vector<std::unique_ptr<solver::State>> newParticles;

    typedef std::unordered_map<TagState, double, solver::State::Hash> WeightMap;
    WeightMap weights;
    double weightTotal = 0;
    GridPosition newRobotPos = decodeGridPosition(obs[0]);
    if (obs[1] == SEEN) {
        // If we saw the opponent, we must be in the same place.
        newParticles.push_back(std::make_unique<TagState>(newRobotPos,
                        newRobotPos, action == TAG));
    } else {
        // We didn't see the opponent, so we must be in different places.
        for (solver::State const *state : previousParticles) {
            TagState const *tagState = static_cast<TagState const *>(state);
            GridPosition oldRobotPos(tagState->getRobotPosition());
            // Ignore states that do not match knowledge of the robot's position.
            if (newRobotPos != getMovedPos(oldRobotPos, action)) {
                continue;
            }
            GridPosition oldOpponentPos(tagState->getOpponentPosition());
            std::vector<TagAction> actions(makeOpponentActions(oldRobotPos,
                            oldOpponentPos));
            std::vector<TagAction> newActions;
            for (TagAction enemyAction : actions) {
                if (getMovedPos(oldOpponentPos, enemyAction) != newRobotPos) {
                    newActions.push_back(enemyAction);
                }
            }
            double probability = 1.0 / newActions.size();
            for (solver::Action enemyAction : newActions) {
                GridPosition newOpponentPos = getMovedPos(oldOpponentPos,
                            enemyAction);
                TagState newState(newRobotPos, newOpponentPos, false);
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
                newParticles.push_back(std::make_unique<TagState>(it->first));
            }
        }
    }
    return newParticles;
}

std::vector<std::unique_ptr<solver::State>> TagModel::generateParticles(
        solver::Action const &action, solver::Observation const &obs) {
    std::vector<std::unique_ptr<solver::State>> newParticles;
    GridPosition newRobotPos = decodeGridPosition(obs[0]);
    if (obs[1] == SEEN) {
        // If we saw the opponent, we must be in the same place.
        newParticles.push_back(
                std::make_unique<TagState>(newRobotPos, newRobotPos,
                        action == TAG));
    } else {
        while (newParticles.size() < getNParticles()) {
            std::unique_ptr<solver::State> state = sampleStateUniform();
            solver::Model::StepResult result = generateStep(*state, action);
            if (obs == result.observation) {
                newParticles.push_back(std::move(result.nextState));
            }
        }
    }
    return newParticles;
}

std::vector<long> TagModel::loadChanges(char const */*changeFilename*/) {
    std::vector<long> result;
    return result;
}

void TagModel::update(long /*time*/,
        std::vector<std::unique_ptr<solver::State>> */*affectedRange*/,
        std::vector<solver::ChangeType> */*typeOfChanges*/) {
}

bool TagModel::modifStSeq(std::vector<solver::State const *> const & /*states*/,
        long /*startAffectedIdx*/, long /*endAffectedIdx*/,
        std::vector<std::unique_ptr<solver::State>> */*modifStSeq*/,
        std::vector<solver::Action> */*modifActSeq*/,
        std::vector<solver::Observation> */*modifObsSeq*/,
        std::vector<double> */*modifRewSeq*/) {
    return false;
}

void TagModel::dispAct(solver::Action const &action, std::ostream &os) {
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
    case TAG:
        os << "TAG";
        break;
    default:
        os << "Invalid action: " << action;
        break;
    }
}

void TagModel::dispObs(solver::Observation const &obs, std::ostream &os) {
    os << decodeGridPosition(obs[0]);
    if (obs[1] == SEEN) {
        os << " SEEN!";
    }
}

void TagModel::dispCell(TagCellType cellType, std::ostream &os) {
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

void TagModel::drawEnv(std::ostream &os) {
    for (std::vector<TagCellType> &row : envMap_) {
        for (TagCellType cellType : row) {
            dispCell(cellType, os);
            os << " ";
        }
        os << endl;
    }
}

void TagModel::drawState(solver::State const &state, std::ostream &os) {
    TagState const *tagState = static_cast<TagState const *>(&state);
    for (std::size_t i = 0; i < envMap_.size(); i++) {
        for (std::size_t j = 0; j < envMap_[0].size(); j++) {
            GridPosition pos(i, j);
            bool hasRobot = (pos == tagState->getRobotPosition());
            bool hasOpponent = (pos == tagState->getOpponentPosition());
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
} /* namespace tag */
