#include "TagModel.hpp"

#include <cmath>
#include <cstddef>

#include <algorithm>
#include <fstream>
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
#include <iterator>
#include <map>
#include <utility>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "ChangeType.hpp"
#include "GlobalResources.hpp"
#include "Model.hpp"
#include "Observation.hpp"
#include "State.hpp"

TagModel::TagModel(po::variables_map vm) {
    // Read the map from the file.
    std::ifstream inFile;
    const char *mapPath = vm["problem.mapPath"].as<std::string>().c_str();
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
    State s;
    cout << "Example States: " << endl;
    for (int i = 0; i < 5; i++) {
        sampleAnInitState(s);
        double q;
        solveHeuristic(s, &q);
        dispState(s, cout);
        cout << " Heuristic: " << q << endl;
    }
    cout << "nParticles: " << nParticles << endl;
    cout << "Environment:" << endl;
    drawEnv(cout);
}

void TagModel::initialise() {
    Coords p;
    nEmptyCells = 0;
    envMap.resize(nRows);
    for (p.i = nRows - 1; p.i >= 0; p.i--) {
        envMap[p.i].resize(nCols);
        for (p.j = 0; p.j < nCols; p.j++) {
            char c = mapText[p.i][p.j];
            long cellType;
            if (c == 'X') {
                cellType = WALL;
            } else {
                cellType = EMPTY + nEmptyCells;
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

long TagModel::encodeCoords(Coords c) {
    return envMap[c.i][c.j];
}

Coords TagModel::decodeCoords(long code) {
    return emptyCells[code];
}

void TagModel::sampleAnInitState(State &sVals) {
    sampleStateUniform(sVals);
}

void TagModel::sampleStateUniform(State &sVals) {
    sVals.vals.resize(nStVars);
    sVals.vals[0] = GlobalResources::randIntBetween(0, nEmptyCells - 1);
    sVals.vals[1] = GlobalResources::randIntBetween(0, nEmptyCells - 1);
    sVals.vals[2] = UNTAGGED;
}

bool TagModel::isTerm(State &sVals) {
    return sVals.vals[2] == TAGGED;
}

void TagModel::solveHeuristic(State &s, double *qVal) {
    Coords robotPos = decodeCoords(s.vals[0]);
    Coords opponentPos = decodeCoords(s.vals[1]);
    if (s.vals[2] == TAGGED) {
        *qVal = 0;
        return;
    }
    int dist = robotPos.distance(opponentPos);
    double nSteps = dist / opponentStayProbability;
    double finalDiscount = std::pow(discount, nSteps);
    *qVal = -moveCost * (1 - finalDiscount) / (1 - discount);
    *qVal += finalDiscount * tagReward;
}

double TagModel::getDefaultVal() {
    return minVal;
}

bool TagModel::makeNextState(State &sVals, unsigned long actId,
        State &nxtSVals) {
    nxtSVals = sVals;
    if (sVals.vals[2] == TAGGED) {
        return false;
    }
    Coords robotPos = decodeCoords(sVals.vals[0]);
    Coords opponentPos = decodeCoords(sVals.vals[1]);
    if (actId == TAG && robotPos == opponentPos) {
        nxtSVals.vals[2] = TAGGED;
        return true;
    }
    moveOpponent(robotPos, opponentPos);
    nxtSVals.vals[1] = encodeCoords(opponentPos);
    robotPos = getMovedPos(robotPos, actId);
    if (!isValid(robotPos)) {
        return false;
    }
    nxtSVals.vals[0] = encodeCoords(robotPos);
    return true;
}

void TagModel::makeOpponentActions(Coords &robotPos, Coords &opponentPos,
        std::vector<long> &actions) {
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
}

void TagModel::moveOpponent(Coords &robotPos, Coords &opponentPos) {
    // Randomize to see if the opponent stays still.
    if (GlobalResources::rand01() < opponentStayProbability) {
        return;
    }
    std::vector<long> actions;
    makeOpponentActions(robotPos, opponentPos, actions);
    Coords newOpponentPos = getMovedPos(opponentPos,
            actions[GlobalResources::randIntBetween(0, actions.size() - 1)]);
    if (isValid(newOpponentPos)) {
        opponentPos = newOpponentPos;
    }
}

Coords TagModel::getMovedPos(Coords &coords, unsigned long actId) {
    Coords movedPos = coords;
    switch (actId) {
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

bool TagModel::isValid(Coords &coords) {
    if (coords.i < 0 || coords.i >= nRows || coords.j < 0 || coords.j >= nCols
            || envMap[coords.i][coords.j] == WALL) {
        return false;
    }
    return true;
}

void TagModel::makeObs(State &nxtSVals, unsigned long /*actId*/,
        Observation &obsVals) {
    obsVals[0] = nxtSVals.vals[0];
    if (nxtSVals.vals[0] == nxtSVals.vals[1]) {
        obsVals[1] = SEEN;
    } else {
        obsVals[1] = UNSEEN;
    }
}

bool TagModel::getNextState(State &sVals, unsigned long actId,
        double *immediateRew, State &nxtSVals, Observation &obs) {
    *immediateRew = getReward(sVals, actId);
    makeNextState(sVals, actId, nxtSVals);
    obs.resize(2);
    makeObs(nxtSVals, actId, obs);
    return isTerm(nxtSVals);
}

double TagModel::getReward(State& /*sVals*/) {
    return 0;
}

double TagModel::getReward(State &sVals, unsigned long actId) {
    if (actId == TAG) {
        if (sVals.vals[0] == sVals.vals[1]) {
            return tagReward;
        } else {
            return -failedTagPenalty;
        }
    } else {
        return -moveCost;
    }
}

void TagModel::getStatesSeeObs(unsigned long actId, Observation &obs,
        std::vector<State> &partSt, std::vector<State> &partNxtSt) {
    std::map<std::vector<double>, double> weights;
    double weightTotal = 0;
    Coords newRobotPos = decodeCoords(obs[0]);
    if (obs[1] == SEEN) {
        State nxtSVals;
        nxtSVals.vals.resize(nStVars);
        nxtSVals.vals[0] = nxtSVals.vals[1] = obs[0];
        nxtSVals.vals[2] = (actId == TAG ? TAGGED : UNTAGGED);
        partNxtSt.push_back(nxtSVals);
        return;
    }
    for (State &sVals : partSt) {
        Coords oldRobotPos = decodeCoords(sVals.vals[0]);
        // Ignore states that do not match knowledge of the robot's position.
        if (newRobotPos != getMovedPos(oldRobotPos, actId)) {
            continue;
        }
        Coords oldOpponentPos = decodeCoords(sVals.vals[1]);
        std::vector<long> actions;
        makeOpponentActions(oldRobotPos, oldOpponentPos, actions);
        std::vector<long> newActions(actions.size());
        std::vector<long>::iterator newActionsEnd = (std::copy_if(
                actions.begin(), actions.end(), newActions.begin(),
                [&oldOpponentPos, &newRobotPos, this] (long action) {
                    return getMovedPos(oldOpponentPos, action) != newRobotPos;
                }));
        newActions.resize(std::distance(newActions.begin(), newActionsEnd));
        double probabilityFactor = 1.0 / newActions.size();
        for (long action : newActions) {
            Coords newOpponentPos = getMovedPos(oldOpponentPos, action);
            State sVals;
            sVals.vals.resize(nStVars);
            sVals.vals[0] = obs[0];
            sVals.vals[1] = encodeCoords(newOpponentPos);
            sVals.vals[2] = UNTAGGED;
            weights[sVals.vals] += probabilityFactor;
            weightTotal += probabilityFactor;
        }
    }
    double scale = nParticles / weightTotal;
    for (std::map<std::vector<double>, double>::iterator it = weights.begin();
            it != weights.end(); it++) {
        double proportion = it->second * scale;
        int numToAdd = std::floor(proportion);
        if (GlobalResources::rand01() <= (proportion - numToAdd)) {
            numToAdd += 1;
        }
        for (int i = 0; i < numToAdd; i++) {
            partNxtSt.emplace_back(it->first);
        }
    }
}

void TagModel::getStatesSeeObs(unsigned long actId, Observation &obs,
        std::vector<State> &partNxtSt) {
    if (obs[1] == SEEN) {
        State nxtSVals;
        nxtSVals.vals.resize(nStVars);
        nxtSVals.vals[0] = nxtSVals.vals[1] = obs[0];
        nxtSVals.vals[2] = (actId == TAG ? TAGGED : UNTAGGED);
        partNxtSt.push_back(nxtSVals);
        return;
    }

    while (partNxtSt.size() < nParticles) {
        State sVals;
        sampleStateUniform(sVals);
        State nxtStVals;
        Observation obs2;
        double reward;
        getNextState(sVals, actId, &reward, nxtStVals, obs2);
        if (obs == obs2) {
            partNxtSt.push_back(nxtStVals);
        }
    }
}

void TagModel::setChanges(const char */*chName*/,
        std::vector<long> &/*chTime*/) {
}

void TagModel::update(long /*tCh*/, std::vector<State> &/*affectedRange*/,
        std::vector<ChangeType> &/*typeOfChanges*/) {
}

bool TagModel::modifStSeq(std::vector<State> &/*seqStVals*/,
        long /*startAffectedIdx*/, long /*endAffectedIdx*/,
        std::vector<State> &/*modifStSeq*/, std::vector<long> &/*modifActSeq*/,
        std::vector<Observation> &/*modifObsSeq*/,
        std::vector<double> &/*modifRewSeq*/) {
    return false;
}

void TagModel::drawEnv(std::ostream &os) {
    for (std::vector<int> &row : envMap) {
        for (int cellType : row) {
            dispCell(cellType, os);
            os << " ";
        }
        os << endl;
    }
}

void TagModel::drawState(State &s, std::ostream &os) {
    for (std::size_t i = 0; i < envMap.size(); i++) {
        for (std::size_t j = 0; j < envMap[0].size(); j++) {
            Coords coords(i, j);
            bool hasRobot = (coords == decodeCoords(s.vals[0]));
            bool hasOpponent = (coords == decodeCoords(s.vals[1]));
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
