#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>

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
	inFile >> mapSize;
    std::string tmp;
	getline(inFile, tmp);
	for (long i = 0; i < mapSize; i++) {
		getline(inFile, tmp);
		envMap.push_back(tmp);
	}
	inFile.close();

    goodRockReward = vm["problem.goodRockReward"].as<double>();
    badRockPenalty = vm["problem.badRockPenalty"].as<double>();
    exitReward = vm["problem.exitReward"].as<double>();
    illegalMovePenalty = vm["problem.illegalMovePenalty"].as<double>();
    halfEfficiencyDistance = vm["problem.halfEfficiencyDistance"].as<double>();
	initialise();

    /*
	cout << "Discount: " << discount << endl;
	cout << "mapSize: " << mapSize << endl;
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

/**
 * Finds and counts the rocks on the map, and initialisese the required
 * data structures and variables.
 */
void RockSampleModel::initialise() {
    nRocks = 0;
    Coords p;
	for (p.i = 0; p.i < mapSize; p.i++) {
		for (p.j = 0; p.j < mapSize; p.j++) {
		    if (envMap[p.i][p.j] == 'S') {
		        startPos = p;
		    } else if (envMap[p.i][p.j] == 'o') {
		        rockCoords.push_back(p);
		        nRocks++;
            }
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
	            s[j+2] = 1;
            } else {
	            s[j+2] = 0;
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

void RockSampleModel::solveHeuristic(StateVals &s, double *qVal) {
    *qVal = 0;
}

bool RockSampleModel::getNextState(StateVals &sVals, long actIdx,
        StateVals &nxtSVals, ObsVals &obs) {
}

double RockSampleModel::getReward(StateVals &sVals) {
}

double RockSampleModel::getReward(StateVals &sVals, long actId) {
}

double RockSampleModel::getNextStateNRew(StateVals &sVals, long actId,
        ObsVals &obs, bool &isTerm) {
}

bool RockSampleModel::getNextState(StateVals &currStVals, long actIdx,
        double *immediateRew, StateVals &nxtSVals, ObsVals &obs) {
}

void RockSampleModel::setChanges(const char* chName, std::vector<long> &chTime)
{
}

void RockSampleModel::update(long tCh, std::vector<StateVals> &affectedRange,
        std::vector<ChType> &typeOfChanges) {
}

double RockSampleModel::getDefaultVal() {
	return minVal;
}

void RockSampleModel::getStatesSeeObs(long actId, ObsVals &obs,
        std::vector<StateVals> &partSt, std::map<int, StateVals> &partNxtSt) {
}

void RockSampleModel::getStatesSeeObs(ObsVals &obs,
        std::vector<StateVals> &posNxtSt) {
}


bool RockSampleModel::isTerm(StateVals &sVals) {
}

bool RockSampleModel::modifStSeq(std::vector<StateVals> &seqStVals,
        long startAffectedIdx, long endAffectedIdx,
		std::vector<StateVals> &modifStSeq,
		std::vector<long> &modifActSeq,
		std::vector<ObsVals> &modifObsSeq,
		std::vector<double> &modifRewSeq) {
}

void RockSampleModel::drawEnv(std::ostream &os) {
    for (std::vector<std::string>:: iterator it = envMap.begin();
            it != envMap.end(); it++) {
        os << *it << std::endl;
    }
}
