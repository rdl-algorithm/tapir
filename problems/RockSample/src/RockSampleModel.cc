#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <set>
#include <cmath>

#include "RockSampleModel.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using std::cout;
using std::cerr;
using std::endl;

RockSampleModel::RockSampleModel(po::variables_map vm) {
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
    goodRockReward = vm["problem.goodRockReward"].as<double>();
    badRockPenalty = vm["problem.badRockPenalty"].as<double>();
    exitReward = vm["problem.exitReward"].as<double>();
    illegalMovePenalty = vm["problem.illegalMovePenalty"].as<double>();
    halfEfficiencyDistance = vm["problem.halfEfficiencyDistance"].as<double>();
    initialise();
    cout << "Constructed the RockSampleModel" << endl;
    cout << "Discount: " << discount << endl;
    cout << "Size: " << nRows << " by " << nCols << endl;
    cout << "Start: " << startPos.i << " " << startPos.j << endl;
    cout << "nRocks: " << nRocks << endl;
    cout << "Rock 0: " << rockCoords[0] << endl;
    cout << "Rock 1: " << rockCoords[1] << endl;
    cout << "good rock reward: " << goodRockReward << endl;
    cout << "nActions: " << nActions << endl;
    cout << "nObservations: " << nObservations << endl;
    cout << "nStVars: " << nStVars << endl;
    cout << "nInitBel: " << nInitBel << endl;
    dispState(initBel[0], cout);
    cout << endl;
    dispState(initBel[1], cout);
    cout << endl;
    dispState(initBel[2], cout);
    cout << endl;
    dispState(initBel[3], cout);
    cout << endl;
    dispState(initBel[255], cout);
    cout << endl;
    cout << "nParticles: " << nParticles << endl;
    cout << "Environment:" << endl;
    drawEnv(cout);
}

void RockSampleModel::initialise() {
    nRocks = 0;
    Coords p;
    for (p.i = 0; p.i < nRows; p.i++) {
        envMap.emplace_back();
        for (p.j = 0; p.j < nCols; p.j++) {
            char c = mapText[p.i][p.j];
            long cellType;
            if (c == 'o') {
                rockCoords.push_back(p);
                cellType = ROCK + nRocks;
                nRocks++;
            } else if (c == 'G') {
                cellType = GOAL;
            } else if (c == 'S') {
                startPos = p;
                cellType = EMPTY;
            } else {
                cellType = EMPTY;
            }
            envMap.back().push_back(cellType);
        }
    }

    nActions = 5 + nRocks;
    nObservations = 2;
    nStVars = 2 + nRocks;
    StateVals s(nStVars);
    s[0] = startPos.i;
    s[1] = startPos.j;
    nInitBel = 1 << nRocks;
    for (long val = 0; val < nInitBel; val++) {
        decodeRocks(val, s);
        initBel.push_back(s);
    }
    minVal = -illegalMovePenalty / (1 - discount);
    maxVal = goodRockReward * nRocks + exitReward;
}

void RockSampleModel::sampleAnInitState(StateVals &sVals) {
    sVals = initBel[GlobalResources::randIntBetween(0, nInitBel - 1)];
}

void RockSampleModel::sampleStateUniform(StateVals &sVals) {
    sVals.resize(nStVars);
    sVals[0] = GlobalResources::randIntBetween(0, nRows - 1);
    sVals[1] = GlobalResources::randIntBetween(0, nCols - 1);
    sampleRocks(sVals);
    cerr << "Uniform random state: ";
    dispState(sVals, cerr);
    cerr << endl;
}

void RockSampleModel::sampleRocks(StateVals &sVals) {
    decodeRocks(GlobalResources::randIntBetween(0, (1 << nRocks) - 1), sVals);
}

void RockSampleModel::decodeRocks(long val, StateVals &sVals) {
    for (int j = 0; j < nRocks; j++) {
        if (val & (1 << j)) {
            sVals[j + 2] = GOOD;
        } else {
            sVals[j + 2] = BAD;
        }
    }
}

bool RockSampleModel::isTerm(StateVals &sVals) {
    return envMap[sVals[0]][sVals[1]] == GOAL;
}

void RockSampleModel::solveHeuristic(StateVals &s, double *qVal) {
    *qVal = 0;
    double currentDiscount = 1;
    Coords currentPos(s[0], s[1]);

    std::set<int> goodRocks;
    for (int i = 0; i < nRocks; i++) {
        if (s[i + 2] == GOOD) {
            goodRocks.insert(i);
        }
    }
    while (!goodRocks.empty()) {
        std::set<int>::iterator it = goodRocks.begin();
        int bestRock = *it;
        long lowestDist = rockCoords[bestRock].distance(currentPos);
        ++it;
        for (; it != goodRocks.end(); ++it) {
            long dist = rockCoords[*it].distance(currentPos);
            if (dist < lowestDist) {
                bestRock = *it;
                lowestDist = dist;
            }
        }
        currentDiscount *= std::pow(discount, lowestDist);
        *qVal += currentDiscount * goodRockReward;
        goodRocks.erase(bestRock);
        currentPos = rockCoords[bestRock];
    }
    currentDiscount *= std::pow(discount, nCols - currentPos.j);
    *qVal += currentDiscount * exitReward;
    // dispState(s, cerr);
    // cerr << endl << "Heuristic: " << *qVal << endl;
}

double RockSampleModel::getDefaultVal() {
    return minVal;
}

bool RockSampleModel::makeNextState(StateVals &sVals, long actId,
        StateVals &nxtSVals) {
    nxtSVals = sVals;
    if (actId >= CHECK) {
        return true;
    }
    if (actId == SAMPLE) {
        int rockNo = envMap[sVals[0]][sVals[1]] - ROCK;
        if (0 <= rockNo && rockNo < nRocks) {
            nxtSVals[2 + rockNo] = BAD;
            return true;
        }
        return false;
    }

    switch (actId) {
    case NORTH:
        nxtSVals[0] -= 1;
        break;
    case EAST:
        nxtSVals[1] += 1;
        break;
    case SOUTH:
        nxtSVals[0] += 1;
        break;
    case WEST:
        nxtSVals[1] -= 1;
    }
    // Check all boundaries.
    if (nxtSVals[0] < 0 || nxtSVals[0] >= nRows || nxtSVals[1] < 0
            || nxtSVals[1] >= nCols) {
        nxtSVals = sVals;
        return false;
    }
    return true;
}

int RockSampleModel::makeObs(StateVals &nxtSVals, long actId) {
    if (actId < CHECK) {
        return NONE;
    }
    int rockNo = actId - CHECK;
    Coords pos(nxtSVals[0], nxtSVals[1]);
    double dist = pos.distance(rockCoords[rockNo]);
    double efficiency = (1 + pow(2, -dist / halfEfficiencyDistance)) * 0.5;
    // cerr << "D: " << dist << " E:" << efficiency << endl;
    if (GlobalResources::rand01() < efficiency) {
        return nxtSVals[2 + rockNo] == GOOD ? GOOD : BAD; // Correct obs.
    } else {
        return nxtSVals[2 + rockNo] == GOOD ? BAD : GOOD; // Incorrect obs.
    }
}

bool RockSampleModel::getNextState(StateVals &sVals, unsigned long actId,
        double *immediateRew, StateVals &nxtSVals, ObsVals &obs) {
    *immediateRew = getReward(sVals, actId);
    makeNextState(sVals, actId, nxtSVals);
    obs.resize(1);
    obs[0] = makeObs(nxtSVals, actId);
    return isTerm(nxtSVals);
}

double RockSampleModel::getReward(StateVals &/*sVals*/) {
    return 0;
}

double RockSampleModel::getReward(StateVals &sVals, unsigned long actId) {
    StateVals nxtSVals;
    bool isLegal = makeNextState(sVals, actId, nxtSVals);
    if (!isLegal) {
        return -illegalMovePenalty;
    }
    if (isTerm(nxtSVals)) {
        return exitReward;
    }

    if (actId == SAMPLE) {
        int rockNo = envMap[sVals[0]][sVals[1]] - ROCK;
        if (0 <= rockNo && rockNo < nRocks) {
            return sVals[2 + rockNo] == GOOD ? goodRockReward : -badRockPenalty;
        } else {
            // We shouldn't end up here, since isLegal should've been false.
            return -illegalMovePenalty;
        }
    }
    return 0;
}

void RockSampleModel::getStatesSeeObs(unsigned long actId, ObsVals &obs,
        std::vector<StateVals> &partSt, std::vector<StateVals> &partNxtSt) {
    // If it's a CHECK action, we condition on the observation.
    if (actId >= CHECK) {
        int rockNo = actId - CHECK;
        std::map<StateVals, double> weights;
        double weightTotal = 0;
        for (StateVals &sv : partSt) {
            Coords pos(sv[0], sv[1]);
            double dist = pos.distance(rockCoords[rockNo]);
            double efficiency = ((1 + pow(2, -dist / halfEfficiencyDistance))
                    * 0.5);
            int rockState = sv[2 + rockNo];
            double probabilityFactor = (
                    rockState == obs[0] ? efficiency : 1 - efficiency);
            weights[sv] += probabilityFactor;
            weightTotal += probabilityFactor;
        }
        double scale = nParticles / weightTotal;
        for (std::map<StateVals, double>::value_type &it : weights) {
            double proportion = it.second * scale;
            int numToAdd = floor(proportion);
            if (GlobalResources::rand01() <= (proportion - numToAdd)) {
                numToAdd += 1;
            }
            partNxtSt.insert(partNxtSt.end(), numToAdd, it.first);
        }
    } else {
        // It's not a CHECK action, so we just add each resultant state.
        for (StateVals &sv : partSt) {
            StateVals nxtStVals;
            makeNextState(sv, actId, nxtStVals);
            partNxtSt.push_back(nxtStVals);
        }
    }
}

void RockSampleModel::getStatesSeeObs(unsigned long actId, ObsVals &obs,
        std::vector<StateVals> &partNxtSt) {
    while (partNxtSt.size() < nParticles) {
        StateVals sVals;
        sampleStateUniform(sVals);
        StateVals nxtStVals;
        ObsVals obs2;
        double reward;
        getNextState(sVals, actId, &reward, nxtStVals, obs2);
        if (obs == obs2) {
            partNxtSt.push_back(nxtStVals);
        }
    }
}

void RockSampleModel::setChanges(const char */*chName*/,
        std::vector<long> &/*chTime*/) {
}

void RockSampleModel::update(long /*tCh*/,
        std::vector<StateVals> &/*affectedRange*/,
        std::vector<Change> &/*typeOfChanges*/) {
}

bool RockSampleModel::modifStSeq(std::vector<StateVals> &/*seqStVals*/,
        long/*startAffectedIdx*/, long/*endAffectedIdx*/,
        std::vector<StateVals> &/*modifStSeq*/,
        std::vector<long> &/*modifActSeq*/,
        std::vector<ObsVals> &/*modifObsSeq*/,
        std::vector<double> &/*modifRewSeq*/) {
    return false;
}

void RockSampleModel::drawEnv(std::ostream &os) {
    for (std::vector<int> &row : envMap) {
        for (int &cellValue : row) {
            dispCell(cellValue, os);
        }
        os << endl;
    }
}

void RockSampleModel::drawState(StateVals &s, std::ostream &os) {
    dispState(s, os);
    os << endl;
    for (size_t i = 0; i < envMap.size(); i++) {
        for (size_t j = 0; j < envMap[0].size(); j++) {
            if (i == s[0] && j == s[1]) {
                os << "x";
                continue;
            }
            dispCell(envMap[i][j], os);
        }
        os << endl;
    }
}
