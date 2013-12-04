#include "RockSampleModel.hpp"

#include <cmath>                        // for pow, floor
#include <cstddef>                      // for size_t

#include <fstream>                      // for ifstream, basic_istream, basic_istream<>::__istream_type
#include <iostream>                     // for cout, cerr
#include <map>                          // for _Rb_tree_const_iterator, map, map<>::value_type
#include <memory>                       // for unique_ptr
#include <set>                          // for set, set<>::iterator
#include <string>                       // for string, getline, char_traits, basic_string
#include <vector>                       // for vector, __alloc_traits<>::value_type, operator==

#include <boost/program_options.hpp>    // for variables_map, variable_value

#include "ChangeType.hpp"               // for ChangeType
#include "GlobalResources.hpp"          // for GlobalResources
#include "Observation.hpp"              // for Observation
#include "RockSampleState.hpp"          // for RockSampleState
#include "State.hpp"                    // for State, operator<

using std::cerr;
using std::cout;
using std::endl;
namespace po = boost::program_options;

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
    cout << "Random initial states:" << endl;
    cout << sampleAnInitState() << endl;
    cout << sampleAnInitState() << endl;
    cout << sampleAnInitState() << endl;
    cout << sampleAnInitState() << endl;

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
    minVal = -illegalMovePenalty / (1 - discount);
    maxVal = goodRockReward * nRocks + exitReward;
}

std::unique_ptr<State> RockSampleModel::sampleAnInitState() {
    long i = global_resources::randIntBetween(0, nRows - 1);
    long j = global_resources::randIntBetween(0, nCols - 1);
    std::vector<bool> rocks = sampleRocks();
    RockSampleState *state = new RockSampleState(GridPosition(i, j), rocks);
    cerr << "Uniform random state: " << state << endl;
    return std::unique_ptr<State>(state);
}

std::unique_ptr<State> RockSampleModel::sampleStateUniform() {
    long i = global_resources::randIntBetween(0, nRows - 1);
    long j = global_resources::randIntBetween(0, nCols - 1);
    std::vector<bool> rocks = sampleRocks();
    RockSampleState *state = new RockSampleState(GridPosition(i, j), rocks);
    cerr << "Uniform random state: " << state << endl;
    return std::unique_ptr<State>(state);
}

std::vector<bool> RockSampleModel::sampleRocks() {
    return decodeRocks(global_resources::randIntBetween(0, (1 << nRocks) - 1));
}

std::vector<bool> RockSampleModel::decodeRocks(long val) {
    std::vector<bool> isRockGood;
    for (int j = 0; j < nRocks; j++) {
        isRockGood.push_back(val &  (1 << j));
    }
    return isRockGood;
}

bool RockSampleModel::isTerm(State &state) {
    const RockSampleState *rockState =
                   dynamic_cast<const RockSampleState*>(&state);
    return envMap[rockState->[0]][sVals[1]] == GOAL;
}

void RockSampleModel::solveHeuristic(RockSampleState &s, double *qVal) {
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

bool RockSampleModel::makeNextState(RockSampleState &sVals, long actId, RockSampleState &nxtSVals) {
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

int RockSampleModel::makeObs(RockSampleState &nxtSVals, long actId) {
    if (actId < CHECK) {
        return NONE;
    }
    int rockNo = actId - CHECK;
    Coords pos(nxtSVals[0], nxtSVals[1]);
    double dist = pos.distance(rockCoords[rockNo]);
    double efficiency = (1 + std::pow(2, -dist / halfEfficiencyDistance)) * 0.5;
    // cerr << "D: " << dist << " E:" << efficiency << endl;
    if (global_resources::rand01() < efficiency) {
        return nxtSVals[2 + rockNo] == GOOD ? GOOD : BAD; // Correct obs.
    } else {
        return nxtSVals[2 + rockNo] == GOOD ? BAD : GOOD; // Incorrect obs.
    }
}

bool RockSampleModel::getNextState(RockSampleState &sVals, unsigned long actId,
        double *immediateRew, RockSampleState &nxtSVals, Observation &obs) {
    *immediateRew = getReward(sVals, actId);
    makeNextState(sVals, actId, nxtSVals);
    obs.resize(1);
    obs[0] = makeObs(nxtSVals, actId);
    return isTerm(nxtSVals);
}

double RockSampleModel::getReward(RockSampleState &/*sVals*/) {
    return 0;
}

double RockSampleModel::getReward(RockSampleState &sVals, unsigned long actId) {
    RockSampleState nxtSVals;
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

void RockSampleModel::getStatesSeeObs(unsigned long actId, Observation &obs,
        std::vector<RockSampleState> &partSt, std::vector<RockSampleState> &partNxtSt) {
    // If it's a CHECK action, we condition on the observation.
    if (actId >= CHECK) {
        int rockNo = actId - CHECK;
        std::map<RockSampleState, double> weights;
        double weightTotal = 0;
        for (RockSampleState &sv : partSt) {
            Coords pos(sv[0], sv[1]);
            double dist = pos.distance(rockCoords[rockNo]);
            double efficiency = ((1
                    + std::pow(2, -dist / halfEfficiencyDistance)) * 0.5);
            int rockState = sv[2 + rockNo];
            double probabilityFactor = (
                    rockState == obs[0] ? efficiency : 1 - efficiency);
            weights[sv] += probabilityFactor;
            weightTotal += probabilityFactor;
        }
        double scale = nParticles / weightTotal;
        for (std::map<RockSampleState, double>::value_type &it : weights) {
            double proportion = it.second * scale;
            int numToAdd = std::floor(proportion);
            if (global_resources::rand01() <= (proportion - numToAdd)) {
                numToAdd += 1;
            }
            partNxtSt.insert(partNxtSt.end(), numToAdd, it.first);
        }
    } else {
        // It's not a CHECK action, so we just add each resultant state.
        for (RockSampleState &sv : partSt) {
            RockSampleState nxtStVals;
            makeNextState(sv, actId, nxtStVals);
            partNxtSt.push_back(nxtStVals);
        }
    }
}

void RockSampleModel::getStatesSeeObs(unsigned long actId, Observation &obs,
        std::vector<RockSampleState> &partNxtSt) {
    while (partNxtSt.size() < nParticles) {
        RockSampleState sVals;
        sampleStateUniform(sVals);
        RockSampleState nxtStVals;
        Observation obs2;
        double reward;
        getNextState(sVals, actId, &reward, nxtStVals, obs2);
        if (obs == obs2) {
            partNxtSt.push_back(nxtStVals);
        }
    }
}

void RockSampleModel::getChangeTimes(const char */*chName*/,
        std::vector<long> &/*chTime*/) {
}

void RockSampleModel::update(long /*tCh*/,
        std::vector<RockSampleState> &/*affectedRange*/,
        std::vector<ChangeType> &/*typeOfChanges*/) {
}

bool RockSampleModel::modifStSeq(std::vector<RockSampleState> &/*seqStVals*/,
        long/*startAffectedIdx*/, long/*endAffectedIdx*/,
        std::vector<RockSampleState> &/*modifStSeq*/, std::vector<long> &/*modifActSeq*/,
        std::vector<Observation> &/*modifObsSeq*/,
        std::vector<double> &/*modifRewSeq*/) {
    return false;
}

void RockSampleModel::drawEnv(std::ostream &os) {
    for (std::vector<int> &row : envMap) {
        for (int cellValue : row) {
            dispCell(cellValue, os);
        }
        os << endl;
    }
}

void RockSampleModel::drawState(RockSampleState &s, std::ostream &os) {
    dispState(s, os);
    os << endl;
    for (std::size_t i = 0; i < envMap.size(); i++) {
        for (std::size_t j = 0; j < envMap[0].size(); j++) {
            if (i == s[0] && j == s[1]) {
                os << "x";
                continue;
            }
            dispCell(envMap[i][j], os);
        }
        os << endl;
    }
}
