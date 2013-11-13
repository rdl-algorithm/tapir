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
using std::endl;

RockSampleModel::RockSampleModel(po::variables_map vm) : Model(vm) {
    // Read the map from the file.
    std::ifstream inFile;
	const char* mapPath = vm["problem.mapPath"].as<std::string>().c_str();
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

    goodRockReward = vm["problem.goodRockReward"].as<double>();
    badRockPenalty = vm["problem.badRockPenalty"].as<double>();
    exitReward = vm["problem.exitReward"].as<double>();
    illegalMovePenalty = vm["problem.illegalMovePenalty"].as<double>();
    halfEfficiencyDistance = vm["problem.halfEfficiencyDistance"].as<double>();
	initialise();

	cout << "Constructed the RockSampleModel" << endl;

    /*
	cout << "Discount: " << discount << endl;
	cout << "Size: " << nRows << " by " << nCols << endl;
	cout << "Start: " << startPos.i << " " << startPos.j << endl;
	cout << "nRocks: " << nRocks << endl;
	cout << rockCoords[0].i << " " << rockCoords[0].j << endl;
	cout << rockCoords[1].i << " " << rockCoords[1].j << endl;
	cout << "good rock reward: " << goodRockReward << endl;
	cout << "nActions: " << nActions << endl;
	cout << "nObservations: " << nObservations << endl;
	cout << "nStVars: " << nStVars << endl;
	cout << "nInitBel: " << nInitBel << endl;
    std::copy(initBel[0].begin(), initBel[0].end(), std::ostream_iterator<double>(cout, " "));
    cout << endl;
    std::copy(initBel[1].begin(), initBel[1].end(), std::ostream_iterator<double>(cout, " "));
    cout << endl;
    std::copy(initBel[2].begin(), initBel[2].end(), std::ostream_iterator<double>(cout, " "));
    cout << endl;
    std::copy(initBel[3].begin(), initBel[3].end(), std::ostream_iterator<double>(cout, " "));
    cout << endl;
    std::copy(initBel[255].begin(), initBel[255].end(), std::ostream_iterator<double>(cout, " "));
    cout << endl;
	cout << "nParticles: " << nParticles << endl;
	*/
}

RockSampleModel::~RockSampleModel() {
    // We don't neeed to do anything here - the vectors go out of scope
    // and are automatically deallocated.
}

void RockSampleModel::initialise() {
    nRocks = 0;
    Coords p;
	for (p.i = 0; p.i < nRows; p.i++) {
	    envMap.emplace_back();
		for (p.j = 0; p.j < nCols; p.j++) {
		    char c = mapText[p.i][p.j];
		    int cellType;
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
            envMap[i].push_back(cellType);
        }
	}

	nActions = 5 + nRocks;
	nObservations = 2;
	nStVars = 2 + nRocks;
	StateVals s(nStVars);
	s[0] = startPos.i;
	s[1] = startPos.j;
	nInitBel = 1 << nRocks;
	for (int val = 0; val < nInitBel; val++) {
	    for (int j = 0; j < nRocks; j++) {
	        if (val & (1 << j)) {
	            s[j+2] = GOOD;
            } else {
	            s[j+2] = BAD;
            }
        }
        initBel.push_back(s);
    }
    minVal = -illegalMovePenalty / (1 - discount);
    maxVal = goodRockReward * nRocks + exitReward;
}


void RockSampleModel::sampleAnInitState(StateVals& tmpStVals) {
	double tmp = GlobalResources::randGen.ranf_arr_next();
	long idx = (long) floor(tmp*(nInitBel-1));
	tmpStVals = initBel[idx];
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
        if (s[i+2]) {
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
        qVal += currentDiscount * goodRockReward;
        goodRocks.erase(bestRock);
        currentPos = rockCoords[bestRock];
    }
    currentDiscount *= std::pow(discount, nCols - currentPos.j);
    qVal += currentDiscount * exitReward;
}

double RockSampleModel::getDefaultVal() {
	return minVal;
}

bool makeNextState(StateVals &sVals, long actId, StateVals &nxtSVals) {
    nxtSVals = sVals;
    switch(actId) {
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
            break;
        case SAMPLE:
            int rockNo = envMap[sVals[0]][sVals[1]] - ROCK;
            if (0 <= rockNo && rockNo < nRocks) {
                nxtSVals[rockNo + 2] = 0;
                return true;
            }
            return false;
    }
    // Check all boundaries.
    if (nxtSVals[0] < 0 || nxtSVals[0] >= nRows || nxtSVals[1] < 0 ||
            nxtSVals[1] >= nCols) {
        nxtSVals = s;
        return false;
    }
    return true;
}

RockSampleObservation RockSampleModel::makeObs(StateVals &sVals, long actId) {
    switch(actId) {
        case NORTH:
        case EAST:
        case SOUTH:
        case WEST:
        case SAMPLE:
            return NONE;
    }
    int rockNo = actId - CHECK;
    Coords pos(sVals[0], sVals[1]);
    double dist = pos.distance(rockCoords[rockNo]);
    double efficiency = (1 + pow(2, -dist / halfEfficiencyDistance)) * 0.5;
	double tmp = GlobalResources::randGen.ranf_arr_next();
	if (tmp <= efficiency) {
	    return sVals[2+rockNo] == GOOD ? GOOD : BAD; // Correct obs.
	} else {
	    return sVals[2+rockNo] == GOOD ? BAD : GOOD; // Incorrect obs.
    }
}


bool RockSampleModel::getNextState(StateVals &sVals, long actId,
        StateVals &nxtSVals, ObsVals &obs) {
    double reward;
    return getNextState(sVals, actId, reward, nxtSvals, obs);
}

bool RockSampleModel::getNextState(StateVals &sVals, long actId,
        double *immediateRew, StateVals &nxtSVals, ObsVals &obs) {
    *immediateRew = getReward(sVals, actId);
    makeNextState(sVals, actId, nxtSVals);
    obs.resize(1);
    obs[0] = makeObs(sVals, actId);
    return isTerm(nxtSVals);
}

double RockSampleModel::getNextStateNRew(StateVals &sVals, long actId,
        ObsVals &obs, bool &isTerm) {
    double reward;
    StateVals nxtSVals;
    isTerm = getNextState(sVals, actId, reward, nxtSvals, obs);
    sVals = nxtSVals;
    return reward;
}

double RockSampleModel::getReward(StateVals &sVals) {
    return 0;
}

double RockSampleModel::getReward(StateVals &sVals, long actId) {
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
            return sVals[rockNo + 2] ? goodRockReward : -badRockPenalty;
        } else {
            // We shouldn't end up here, since isLegal should've been false.
            return -illegalMovePenalty;
        }
    }
    return 0;
}



void RockSampleModel::getStatesSeeObs(long actId, ObsVals &obs,
        std::vector<StateVals> &partSt, std::map<int, StateVals> &partNxtSt) {
    // If it's a CHECK action, we condition on the observation.
    if (actId >= CHECK) {
        int rockNo = actId - CHECK;
        Coords pos(sVals[0], sVals[1]);
        double dist = pos.distance(rockCoords[rockNo]);
        double efficiency = (1 + pow(2, -dist / halfEfficiencyDistance)) * 0.5;
        std::map<StateVals, double> weights;
        double weightTotal = 0;
        for (std::vector<StateVals>::iterator it = partSt.begin();
                it != partSt.end(); it++) {
            int rockState = *it[2+rockNo];
            double probabilityFactor = 2 * (rockState == obs[0] ? efficiency :
                    1 - efficiency);
            weights[*it] += probabilityFactor;
            weightTotal += probabilityFactor;
        }
        double scale = nParticles / weightTotal;
        for (std::map<StateVals, double>::iterator it = weights.begin();
                it != weights.end(); it++) {
            double proportion = *it.second * scale;
            int numToAdd = floor(proportion);
            double tmp = GlobalResources::randGen.ranf_arr_next();
            if (tmp <= (proportion - numToAdd)) {
                numToAdd += 1;
            }
            partNxtSt.insert(partNxtSt.end(), numToAdd, *it.first);
        }
    } else {
        // It's not a CHECK action, so we just add each resultant state.
        for (std::vector<StateVals>::iterator it = partSt.begin();
                it != partSt.end(); it++) {
            StateVals nextState;
            makeNextState(*it, actId, nextState);
            partNxtSt.push_back(nextState);
        }
    }
}

void RockSampleModel::getStatesSeeObs(ObsVals &obs,
        std::vector<StateVals> &posNxtSt) {
}


void RockSampleModel::setChanges(const char* chName,
        std::vector<long> &chTime) {
}

void RockSampleModel::update(long tCh, std::vector<StateVals> &affectedRange,
        std::vector<ChType> &typeOfChanges) {
}

bool RockSampleModel::modifStSeq(std::vector<StateVals> &seqStVals,
        long startAffectedIdx, long endAffectedIdx,
		std::vector<StateVals> &modifStSeq,
		std::vector<long> &modifActSeq,
		std::vector<ObsVals> &modifObsSeq,
		std::vector<double> &modifRewSeq) {
}


void RockSampleModel::drawEnv(std::ostream &os) {
    for (std::vector<std::string>:: iterator it = mapText.begin();
            it != mapText.end(); it++) {
        os << *it << std::endl;
    }
}
