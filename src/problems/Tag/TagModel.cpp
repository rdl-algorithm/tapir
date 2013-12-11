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
#include "problems/GridPosition.hpp"    // for GridPosition, operator==, operator!=, operator<<
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

TagModel::TagModel(RandomGenerator *randGen, po::variables_map vm) : Model(
            randGen) {
    // Read the map from the file.
    std::ifstream inFile;
    char const *mapPath = vm["problem.mapPath"].as<std::string>().c_str();
    inFile.open(mapPath);
    if (!inFile.is_open()) {
        std::cerr << "Fail to open " << mapPath << "\n";
        exit(1);
    }
    inFile >> nRows >> nCols;
    std::string tmp;
    getline(inFile, tmp);
    for (long i = 0; i < nRows; i++) {
        getline(inFile, tmp);
        mapText.push_back(tmp);
    }
    inFile.close();

    nParticles = vm["SBT.nParticles"].as<long>();
    maxTrials = vm["SBT.maxTrials"].as<long>();
    maxDistTry = vm["SBT.maxDistTry"].as<long>();

    exploreCoef = vm["SBT.exploreCoef"].as<double>();
    depthTh = vm["SBT.depthTh"].as<double>();
    distTh = vm["SBT.distTh"].as<double>();

    discount = vm["problem.discount"].as<double>();
    moveCost = vm["problem.moveCost"].as<double>();
    tagReward = vm["problem.tagReward"].as<double>();
    failedTagPenalty = vm["problem.failedTagPenalty"].as<double>();
    opponentStayProbability =
        vm["problem.opponentStayProbability"].as<double>();
    initialise();
    cout << "Constructed the TagModel" << endl;
    cout << "Discount: " << discount << endl;
    cout << "Size: " << nRows << " by " << nCols << endl;
    cout << "move cost: " << moveCost << endl;
    cout << "nActions: " << nActions << endl;
    cout << "nObservations: " << nObservations << endl;
    cout << "nStVars: " << nStVars << endl;
    cout << "Example States: " << endl;
    for (int i = 0; i < 5; i++) {
        std::unique_ptr<State> state = sampleAnInitState();
        cout << *state << " Heuristic: " << solveHeuristic(*state) << endl;
    }
    cout << "nParticles: " << nParticles << endl;
    cout << "Environment:" << endl;
    drawEnv(cout);
}

void TagModel::initialise() {
    GridPosition p;
    nEmptyCells = 0;
    envMap.resize(nRows);
    for (p.i = nRows - 1; p.i >= 0; p.i--) {
        envMap[p.i].resize(nCols);
        for (p.j = 0; p.j < nCols; p.j++) {
            char c = mapText[p.i][p.j];
            CellType cellType;
            if (c == 'X') {
                cellType = WALL;
            } else {
                cellType = (CellType)(EMPTY + nEmptyCells);
                emptyCells.push_back(p);
                nEmptyCells++;
            }
            envMap[p.i][p.j] = cellType;
        }
    }

    nActions = 5;
    nObservations = nEmptyCells * 2;
    nStVars = 3;
    minVal = -failedTagPenalty / (1 - discount);
    maxVal = tagReward;
}

long TagModel::encodeGridPosition(GridPosition pos) {
    return envMap[pos.i][pos.j];
}

GridPosition TagModel::decodeGridPosition(long code) {
    return emptyCells[code];
}

std::unique_ptr<State> TagModel::sampleAnInitState() {
    return sampleStateUniform();
}

std::unique_ptr<State> TagModel::sampleStateUniform() {
    std::uniform_int_distribution<long> randomCell(0, nEmptyCells - 1);
    GridPosition robotPos = decodeGridPosition(randomCell(*randGen));
    GridPosition opponentPos = decodeGridPosition(randomCell(*randGen));
    return std::make_unique<TagState>(robotPos, opponentPos, false);
}

bool TagModel::isTerm(State const &state) {
    return static_cast<TagState const *>(&state)->isTagged();
}

double TagModel::solveHeuristic(State const &state) {
    TagState const *tagState = static_cast<TagState const *>(&state);
    if (tagState->isTagged()) {
        return 0;
    }
    GridPosition robotPos = tagState->getRobotPosition();
    GridPosition opponentPos = tagState->getOpponentPosition();
    int dist = robotPos.manhattanDistanceTo(opponentPos);
    double nSteps = dist / opponentStayProbability;
    double finalDiscount = std::pow(discount, nSteps);
    double qVal = -moveCost * (1 - finalDiscount) / (1 - discount);
    qVal += finalDiscount * tagReward;
    return qVal;
}

double TagModel::getDefaultVal() {
    return minVal;
}

std::pair<std::unique_ptr<TagState>, bool> TagModel::makeNextState(
        State const &state, Action const &action) {
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
    if (std::bernoulli_distribution(opponentStayProbability)(*randGen)) {
        return opponentPos;
    }
    std::vector<TagAction> actions(makeOpponentActions(robotPos, opponentPos));;
    Action action = actions[std::uniform_int_distribution<long>(
                                0, actions.size() - 1)(*randGen)];
    GridPosition newOpponentPos = getMovedPos(opponentPos, action);
    if (!isValid(newOpponentPos)) {
        newOpponentPos = opponentPos;
    }
    return newOpponentPos;
}

GridPosition TagModel::getMovedPos(GridPosition const &position,
        Action const &action) {
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
    }
    return movedPos;
}

bool TagModel::isValid(GridPosition const &position) {
    return (position.i >= 0 && position.i < nRows && position.j >= 0
            && position.j < nCols && envMap[position.i][position.j] != WALL);
}

Observation TagModel::makeObs(Action const & /*action*/,
        TagState const &state) {
    Observation obs(2);
    obs[0] = encodeGridPosition(state.getRobotPosition());
    if (state.getRobotPosition() == state.getOpponentPosition()) {
        obs[1] = SEEN;
    } else {
        obs[1] = UNSEEN;
    }
    return obs;
}

Model::StepResult TagModel::generateStep(State const &state,
        Action const &action) {
    Model::StepResult result;
    result.action = action;
    std::unique_ptr<TagState> nextState = makeNextState(state, action).first;

    result.observation = makeObs(action, *nextState);
    result.immediateReward = getReward(state, action);
    result.isTerminal = isTerm(*nextState);
    result.nextState = std::move(nextState);
    return result;
}

double TagModel::getReward(State const & /*state*/) {
    return 0;
}

double TagModel::getReward(State const &state, Action const &action) {
    if (action == TAG) {
        TagState const *tagState = static_cast<TagState const *>(&state);
        if (tagState->getRobotPosition() == tagState->getOpponentPosition()) {
            return tagReward;
        } else {
            return -failedTagPenalty;
        }
    } else {
        return -moveCost;
    }
}


std::vector<std::unique_ptr<State>> TagModel::generateParticles(
        Action const &action, Observation const &obs,
        std::vector<State *> const &previousParticles) {
    std::vector<std::unique_ptr<State>> newParticles;

    typedef std::unordered_map<TagState, double, State::Hash> WeightMap;
    WeightMap weights;
    double weightTotal = 0;
    GridPosition newRobotPos = decodeGridPosition(obs[0]);
    if (obs[1] == SEEN) {
        // If we saw the opponent, we must be in the same place.
        newParticles.push_back(std::make_unique<TagState>(newRobotPos,
                        newRobotPos, action == TAG));
    } else {
        // We didn't see the opponent, so we must be in different places.
        for (State const *state : previousParticles) {
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
            for (TagAction action : actions) {
                if (getMovedPos(oldOpponentPos, action) != newRobotPos) {
                    newActions.push_back(action);
                }
            }
            double probability = 1.0 / newActions.size();
            for (Action action : newActions) {
                GridPosition newOpponentPos = getMovedPos(oldOpponentPos,
                            action);
                TagState state(newRobotPos, newOpponentPos, false);
                weights[state] += probability;
                weightTotal += probability;
            }
        }
        double scale = nParticles / weightTotal;
        for (WeightMap::iterator it = weights.begin(); it != weights.end();
             it++) {
            double proportion = it->second * scale;
            int numToAdd = std::floor(proportion);
            if (std::bernoulli_distribution(proportion-numToAdd)(*randGen)) {
                numToAdd += 1;
            }
            for (int i = 0; i < numToAdd; i++) {
                newParticles.push_back(std::make_unique<TagState>(it->first));
            }
        }
    }
    return newParticles;
}

std::vector<std::unique_ptr<State>> TagModel::generateParticles(
        Action const &action, Observation const &obs) {
    std::vector<std::unique_ptr<State>> newParticles;
    GridPosition newRobotPos = decodeGridPosition(obs[0]);
    if (obs[1] == SEEN) {
        // If we saw the opponent, we must be in the same place.
        newParticles.push_back(
                std::make_unique<TagState>(newRobotPos, newRobotPos,
                        action == TAG));
    } else {
        while (newParticles.size() < nParticles) {
            std::unique_ptr<State> state = sampleStateUniform();
            Model::StepResult result = generateStep(*state, action);
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
        std::vector<std::unique_ptr<State>> */*affectedRange*/,
        std::vector<ChangeType> */*typeOfChanges*/) {
}

bool TagModel::modifStSeq(std::vector<State const *> const & /*states*/,
        long /*startAffectedIdx*/, long /*endAffectedIdx*/,
        std::vector<std::unique_ptr<State>> */*modifStSeq*/,
        std::vector<Action> */*modifActSeq*/,
        std::vector<Observation> */*modifObsSeq*/,
        std::vector<double> */*modifRewSeq*/) {
    return false;
}

void TagModel::dispAct(Action const &action, std::ostream &os) {
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
    }
}

void TagModel::dispObs(Observation const &obs, std::ostream &os) {
    os << decodeGridPosition(obs[0]);
    if (obs[1] == SEEN) {
        os << " SEEN!";
    }
}

void TagModel::dispCell(CellType cellType, std::ostream &os) {
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
    for (std::vector<CellType> &row : envMap) {
        for (CellType cellType : row) {
            dispCell(cellType, os);
            os << " ";
        }
        os << endl;
    }
}

void TagModel::drawState(State const &state, std::ostream &os) {
    TagState const *tagState = static_cast<TagState const *>(&state);
    for (std::size_t i = 0; i < envMap.size(); i++) {
        for (std::size_t j = 0; j < envMap[0].size(); j++) {
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
                if (envMap[i][j] == WALL) {
                    os << "X";
                } else {
                    os << ".";
                }
            }
        }
        os << endl;
    }
}

