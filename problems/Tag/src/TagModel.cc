#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <set>
#include <cmath>

#include "TagModel.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using std::cout;
using std::cerr;
using std::endl;

TagModel::TagModel(po::variables_map vm) :
        Model(vm) {
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
    StateVals s;
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

TagModel::~TagModel() {
    // We don't neeed to do anything here - the vectors go out of scope
    // and are automatically deallocated.
}

void TagModel::initialise() {
    Coords p;
    nEmptyCells = 0;
    envMap.resize(nRows);
    for (p.i = nRows - 1; p.i >= 0; p.i--) {
        envMap[p.i].resize(nCols);
        for (p.j = nCols - 1; p.j >= 0; p.j--) {
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

void TagModel::sampleAnInitState(StateVals &sVals) {
    sampleStateUniform(sVals);
}

void TagModel::sampleStateUniform(StateVals &sVals) {
    sVals.resize(nStVars);
    sVals[0] = GlobalResources::randIntBetween(0, nEmptyCells);
    sVals[1] = GlobalResources::randIntBetween(0, nEmptyCells);
    sVals[2] = UNTAGGED;
}

bool TagModel::isTerm(StateVals &sVals) {
    return sVals[2] == TAGGED;
}

void TagModel::solveHeuristic(StateVals &s, double *qVal) {
    Coords robotPos = decodeCoords(s[0]);
    Coords opponentPos = decodeCoords(s[1]);
    if (s[2] == TAGGED) {
        *qVal = 0;
        return;
    }
    int dist = robotPos.distance(opponentPos);
    double nSteps = dist / opponentStayProbability;
    double finalDiscount = pow(discount, nSteps);
    *qVal = -moveCost * (1 - finalDiscount) / (1 - discount);
    *qVal += finalDiscount * tagReward;
}

double TagModel::getDefaultVal() {
    return minVal;
}

bool TagModel::makeNextState(StateVals &sVals, long actId,
        StateVals &nxtSVals) {
    nxtSVals = sVals;
    if (sVals[2] == TAGGED) {
        return false;
    }

    Coords robotPos = decodeCoords(sVals[0]);
    Coords opponentPos = decodeCoords(sVals[1]);
    if (actId == TAG && robotPos == opponentPos) {
        nxtSVals[2] = TAGGED;
        return true;
    }
    moveOpponent(robotPos, opponentPos);
    nxtSVals[1] = encodeCoords(opponentPos);
    robotPos = getMovedPos(robotPos, actId);
    if (!isValid(robotPos)) {
        return false;
    }
    nxtSVals[0] = encodeCoords(robotPos);
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

Coords TagModel::getMovedPos(Coords &coords, long actId) {
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

void TagModel::makeObs(StateVals &nxtSVals, long actId, ObsVals &obsVals) {
    obsVals[0] = nxtSVals[0];
    if (nxtSVals[0] == nxtSVals[1]) {
        obsVals[1] = SEEN;
    } else {
        obsVals[1] = UNSEEN;
    }
}

bool TagModel::getNextState(StateVals &sVals, long actId, double *immediateRew,
        StateVals &nxtSVals, ObsVals &obs) {
    *immediateRew = getReward(sVals, actId);
    makeNextState(sVals, actId, nxtSVals);
    obs.resize(2);
    makeObs(sVals, actId, obs);
    return isTerm(nxtSVals);
}

double TagModel::getReward(StateVals &sVals) {
    return 0;
}

double TagModel::getReward(StateVals &sVals, long actId) {
    if (actId == TAG) {
        if (sVals[0] == sVals[1]) {
            return tagReward;
        } else {
            return -failedTagPenalty;
        }
    } else {
        return -moveCost;
    }
}

void TagModel::getStatesSeeObs(long actId, ObsVals &obs,
        std::vector<StateVals> &partSt, std::vector<StateVals> &partNxtSt) {
    std::map<StateVals, double> weights;
    double weightTotal = 0;
    Coords newRobotPos = decodeCoords(obs[0]);
    if (obs[1] == SEEN) {
        Coords newOpponentPos = newRobotPos;
        StateVals nxtSVals(nStVars);
        nxtSVals[0] = nxtSVals[1] = obs[0];
        nxtSVals[2] = (actId == TAG ? TAGGED : UNTAGGED);
        partNxtSt.push_back(nxtSVals);
        return;
    }
    for (StateVals &sv : partSt) {
        Coords oldRobotPos = decodeCoords(sv[0]);
        // Ignore states that do not match knowledge of the robot's position.
        if (newRobotPos != getMovedPos(oldRobotPos, actId)) {
            continue;
        }
        Coords oldOpponentPos = decodeCoords((sv)[1]);
        std::vector<long> actions;
        makeOpponentActions(oldRobotPos, oldOpponentPos, actions);
        std::vector<long> newActions(actions.size());
        auto newActionsEnd = std::copy_if(actions.begin(), actions.end(),
                newActions.begin(), [](long x) {
                    return getMovedPos(oldOpponentPos, x) != newRobotPos;
                });
        newActions.resize(std::distance(newActions.begin(), newActionsEnd));
        for (auto &it2 : newActions) {
            double probabilityFactor = 1.0 / newActions.size();
        }
    }
    double scale = nParticles / weightTotal;
    for (std::map<StateVals, double>::iterator it = weights.begin();
            it != weights.end(); it++) {
        double proportion = it->second * scale;
        int numToAdd = floor(proportion);
        if (GlobalResources::rand01() <= (proportion - numToAdd)) {
            numToAdd += 1;
        }
        partNxtSt.insert(partNxtSt.end(), numToAdd, it->first);
    }
}

void TagModel::getStatesSeeObs(long actId, ObsVals &obs,
        std::vector<StateVals> &partNxtSt) {
    if (obs[1] == SEEN) {
        Coords newOpponentPos = decodeCoords(obs[0]);
        StateVals nxtSVals(nStVars);
        nxtSVals[0] = nxtSVals[1] = obs[0];
        nxtSVals[2] = (actId == TAG ? TAGGED : UNTAGGED);
        partNxtSt.push_back(nxtSVals);
        return;
    }
}

void TagModel::setChanges(const char *chName, std::vector<long> &chTime) {
}

void TagModel::update(long tCh, std::vector<StateVals> &affectedRange,
        std::vector<Change> &typeOfChanges) {
}

bool TagModel::modifStSeq(std::vector<StateVals> &seqStVals,
        long startAffectedIdx, long endAffectedIdx,
        std::vector<StateVals> &modifStSeq, std::vector<long> &modifActSeq,
        std::vector<ObsVals> &modifObsSeq, std::vector<double> &modifRewSeq) {
    return false;
}

void TagModel::drawEnv(std::ostream &os) {
    for (std::vector<int> &row : envMap) {
        for (int &cellType : row) {
            dispCell(cellType, os);
            os << " ";
        }
        os << endl;
    }
}
