#include "UnderwaterNavModel.hpp"

#include <cctype>                       // for islower
#include <climits>                      // for LONG_MAX
#include <cmath>                        // for floor, pow, abs, sqrt
#include <cstdlib>                      // for exit

#include <fstream>                      // for basic_istream, basic_istream<>::__istream_type, ifstream, basic_istream::operator>>
#include <iostream>                     // for cerr, cout
#include <map>                          // for map, _Rb_tree_iterator, map<>::mapped_type, __alloc_traits<>::value_type, map<>::iterator
#include <sstream>                      // for stringstream
#include <string>                       // for string, operator>>, operator<<, char_traits, getline, basic_string
#include <utility>                      // for pair
#include <vector>                       // for vector, vector<>::iterator

#include "defs.hpp"                     // for RandomGenerator
#include "solver/ChangeType.hpp"               // for ChangeType, ChangeType::REWARD, ChangeType::ADDOBSERVATION, ChangeType::ADDOBSTACLE
#include "solver/Observation.hpp"              // for Observation
#include "solver/State.hpp"                    // for State
#include "StRoadmap.hpp"                // for StRoadmap

using std::cerr;
using std::cout;
using std::endl;
using std::string;
namespace po = boost::program_options;

namespace uwnav {
#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Weffc++"
UnderwaterNavModel::UnderwaterNavModel(RandomGenerator *randGen, po::variables_map vm) : Model(randGen) {
#pragma GCC diagnostic pop
    std::ifstream inFile;

    char const *mapPath = vm["problem.mapPath"].as<string>().c_str();
    inFile.open(mapPath);
    if (!inFile.is_open()) {
        cerr << "Fail to open " << mapPath << "\n";
        std::exit(1);
    }

    string tmp;
    inFile >> nX_ >> nY_;
    std::getline(inFile, tmp);

    for (unsigned long i = 0; i < nY_; i++) {
        std::getline(inFile, tmp);
        mapText_.push_back(tmp);
    }

    inFile >> tmp >> nSpcRew_;
    spcRew_.resize(nSpcRew_);
    for (short i = 0; i < nSpcRew_; i++) {
        inFile >> tmp >> spcRew_[i];
    }

    inFile.close();

    nParticles = vm["SBT.nParticles"].as<long>();
    maxTrials = vm["SBT.maxTrials"].as<long>();
    maxNnComparisons = vm["SBT.maxNnComparisons"].as<long>();

    heuristicExploreCoefficient = vm["SBT.heuristicExploreCoefficient"].as<double>();
    minimumDiscount = vm["SBT.minimumDiscount"].as<double>();
    maxNnDistance = vm["SBT.maxNnDistance"].as<double>();

    discount_ = vm["problem.discount"].as<double>();
    goalReward_ = vm["problem.goalReward"].as<double>();
    crashPenalty_ = vm["problem.crashPenalty"].as<double>();
    moveCost_ = vm["problem.moveCost"].as<double>();
    ctrlCorrectProb_ = vm["problem.ctrlCorrectProb"].as<double>();

    rolloutExploreTh_ = vm["heuristic.rolloutExploreTh"].as<double>();
    nVerts_ = vm["heuristic.nVerts"].as<long>();
    nTryCon_ = vm["heuristic.nTryCon"].as<long>();
    maxDistCon_ = vm["heuristic.maxDistCon"].as<long>();

    setInitObsGoal();

    moveDiagCost_ = std::sqrt(2) * moveCost_;
    ctrlErrProb1_ = ((1.0 - ctrlCorrectProb_) / 2.0) + ctrlCorrectProb_;
    minVal_ = crashPenalty_ / (1.0 - discount_);
    maxVal_ = goalReward_;

    roadmap_ = new StRoadmap(randGen, goals_, nVerts_, nGoals_, nTryCon_, maxDistCon_,
                            cellType_, nX_, nY_);
}

void UnderwaterNavModel::setInitObsGoal() {
    nActions_ = 5;
    nObservations_ = 0;
    nStVars_ = 2;
    nInitBel_ = 0;

    nGoals_ = 0;
    long i = 0;
    for (string &line : mapText_) {
        for (unsigned long j = 0; j < nX_; j++) {
            if (line[j] == 'S') {
                UnderwaterNavState s(2);
                s[0] = j;
                s[1] = i;
                initBel_.push_back(s);
                nInitBel_++;
            } else if (line[j] == 'O') {
                cellType_[j][i] = 3;
                UnderwaterNavState s(2);
                s[0] = j;
                s[1] = i;
                allObservations_.push_back(s);
                nObservations_++;
            } else if (line[j] == 'D') {
                cellType_[j][i] = 1;
                UnderwaterNavState s(2);
                s[0] = j;
                s[1] = i;
                goals_.push_back(s);
                nGoals_++;
            } else if (line[j] == 'R') {
                cellType_[j][i] = 2;
                UnderwaterNavState s(2);
                s[0] = j;
                s[1] = i;
                rocks_.push_back(s);
                nRocks_++;
            } else if (line[j] == '1') {
                cellType_[j][i] = 5;
            } else if (std::islower(line[j])) {
                cellType_[j][i] = (short) (line[j]) - 97;
            }
        }
        i++;
    }
}

void UnderwaterNavModel::sampleAnInitState(UnderwaterNavState &tmpStVals) {
    long idx = global_resources::randIntBetween(0, nInitBel_ - 1);
    tmpStVals = initBel_[idx];
//cerr << "sampled " << tmpStVals[0] << " " << tmpStVals[1] << endl;
}

/*
 long UnderwaterNavModel::getRolloutAct(State &stVals) {
 double tmp = GlobalResources::rand01();
 if (tmp < rolloutExploreTh) {   // Random
 return GlobalResources::randIntBetween(0, nActions - 1);
 }
 else {                          // Shortest path to goal.
 long bestAct = 0;
 double tmpDist;
 double minDist = getExpDist(stVals, 0);
 for (long i = 1; i < nActions; i++) {
 tmpDist = getExpDist(stVals, i);
 if (tmpDist < minDist) {
 minDist = tmpDist;
 bestAct = i;
 }
 }
 return bestAct;
 }
 }

 long UnderwaterNavModel::getRolloutAct(State &stVals, double *rew) {
 long selectedActIdx;
 double tmp = GlobalResources::rand01()
 if (tmp < rolloutExploreTh) {   // Random
 tmp = GlobalResources::randIntBetween(0, nActions - 1);
 }
 else {                          // Shortest path to goal.
 long bestAct = 0;
 double tmpDist;
 double minDist = getExpDist(stVals, 0);
 for (long i = 1; i < nActions; i++) {
 tmpDist = getExpDist(stVals, i);
 if (tmpDist < minDist) {
 minDist = tmpDist;
 bestAct = i;
 }
 }
 selectedActIdx = bestAct;
 }
 if (actId == EAST || actId == NORTH || actId == SOUTH) {
 *rew = moveCost;
 }
 else {
 *rew = moveDiagCost;
 }
 return selectedActIdx;
 }
 */

// Nearest distance to goal.
void UnderwaterNavModel::solveHeuristic(UnderwaterNavState &s, double *qVal) {
    UnderwaterNavState nxtS;
    if (inRock(s)) {
        *qVal = crashPenalty_;
    } else {
        *qVal = 0;
    }
    double dist = roadmap_->getDistToGoal(s);
    //double dist = getDistToNearestObs(s, nxtS);
    //dist = dist + getDistToNearestGoal(nxtS);
    if (dist < LONG_MAX) {
        *qVal = *qVal + std::pow(discount_, dist) * goalReward_
                + moveCost_ * (1 - std::pow(discount_, dist + 1))
                / (1 - discount_);
    } else {
        *qVal = *qVal + crashPenalty_ / (1 - discount_);
    }
}

double UnderwaterNavModel::getDistToNearestGoal(UnderwaterNavState &st) {
    std::vector<UnderwaterNavState>::iterator itGoal = goals_.begin();
    double minDist = getDist(st, *itGoal);
    double tmpDist;
    itGoal++;
    for (; itGoal != goals_.end(); itGoal++) {
        tmpDist = getDist(st, *itGoal);
        if (tmpDist < minDist) {
            minDist = tmpDist;
        }
    }
    return minDist;
}

double UnderwaterNavModel::getDistToNearestObs(UnderwaterNavState &st, UnderwaterNavState &nxtSt) {
    std::vector<UnderwaterNavState>::iterator it = allObservations_.begin();
    double minDist = getDist(st, *it);
    nxtSt = *it;
    double tmpDist;
    it++;
    for (; it != allObservations_.end(); it++) {
        tmpDist = getDist(st, *it);
        if (tmpDist < minDist) {
            minDist = tmpDist;
            nxtSt = *it;
        }
    }
    return minDist;
}

/*
 double UnderwaterNavModel::getExpDist(State &s, long firstAct) {
 double totDist = 0.0;
 std::vector<State>::iterator itGoal;
 for (long i = 0; i < nParticles; i++) {
 State nxtSt;
 getNextState(s, firstAct, nxtSt);

 itGoal = goals.begin();
 double minDist = getDist(nxtSt, *itGoal);
 double tmpDist;
 itGoal ++;
 for (; itGoal != goals.end(); itGoal++) {
 tmpDist = getDist(nxtSt, *itGoal);
 if (tmpDist < minDist) {
 minDist = tmpDist;
 }
 }
 totDist = totDist + minDist;
 }
 return totDist / (double) nParticles;
 }
 */
void UnderwaterNavModel::getNextState(UnderwaterNavState &s, unsigned long actId,
        UnderwaterNavState &sp) {
    sp.resize(2);

    double tmp = global_resources::rand01();
    switch (actId) {
    case EAST: {
        if (tmp < ctrlCorrectProb_) {
            sp[0] = s[0] + 1.0;
            sp[1] = s[1];
        } else if (tmp < ctrlErrProb1_) {
            sp[0] = s[0] + 1.0;
            sp[1] = s[1] + 1.0;
        } else {
            sp[0] = s[0] + 1.0;
            sp[1] = s[1] - 1.0;
        }
        break;
    }
    case NORTH: {
        if (tmp < ctrlCorrectProb_) {
            sp[0] = s[0];
            sp[1] = s[1] - 1.0;
        } else if (tmp < ctrlErrProb1_) {
            sp[0] = s[0] - 1.0;
            sp[1] = s[1] - 1.0;
        } else {
            sp[0] = s[0] + 1.0;
            sp[1] = s[1] - 1.0;
        }
        break;
    }
    case SOUTH: {
        if (tmp < ctrlCorrectProb_) {
            sp[0] = s[0];
            sp[1] = s[1] + 1.0;
        } else if (tmp < ctrlErrProb1_) {
            sp[0] = s[0] - 1.0;
            sp[1] = s[1] + 1.0;
        } else {
            sp[0] = s[0] + 1.0;
            sp[1] = s[1] + 1.0;
        }
        break;
    }
    case NORTHEAST: {
        if (tmp < ctrlCorrectProb_) {
            sp[0] = s[0] + 1.0;
            sp[1] = s[1] - 1.0;
        } else if (tmp < ctrlErrProb1_) {
            sp[0] = s[0];
            sp[1] = s[1] - 1.0;
        } else {
            sp[0] = s[0] + 1.0;
            sp[1] = s[1];
        }
        break;
    }
    case SOUTHEAST: {
        if (tmp < ctrlCorrectProb_) {
            sp[0] = s[0] + 1.0;
            sp[1] = s[1] + 1.0;
        } else if (tmp < ctrlErrProb1_) {
            sp[0] = s[0];
            sp[1] = s[1] + 1.0;
        } else {
            sp[0] = s[0] + 1.0;
            sp[1] = s[1];
        }
        break;
    }
    }
//cerr << "sp: " << s[0] << " " << s[1] << " act " << actId << " s " << sp[0] << " " << sp[1] << endl;
    if (sp[0] < 0 || sp[0] >= nX_ || sp[1] < 0 || sp[1] >= nY_) {
        sp = s;
    }
    /*
     if (cellType[sp[0]][sp[1]] == 5) {
     sp = s;
     }
     */
    /*
     if (cellType[sp[0]][sp[1]] == 2) {
     cerr << "Visit rocks at " << sp[0] << " " << sp[1] << endl;
     }
     */
    std::map<long, std::map<long, short> >::iterator itCellType = cellType_.find(
                sp[0]);
    if (itCellType != cellType_.end()) {
        if (itCellType->second.find(sp[1]) != itCellType->second.end()) {
            if (cellType_[sp[0]][sp[1]] == 5) {
                sp = s;
            }
        }
    }

//cout << "s: " << s[0] << " " << s[1] << " s': " << sp[0] << " " << sp[1] << " act: "
//      << actId << " tmp " << tmp << " nX: " << nX << " " << nY << endl;
}

double UnderwaterNavModel::getDist(UnderwaterNavState &s1, UnderwaterNavState &s2) {
    double d = 0.0;
    std::vector<double>::iterator it1, it2;
    for (it1 = s1.begin(), it2 = s2.begin(); it1 != s1.end(); it1++, it2++) {
        d = d + std::abs(*it1 - *it2);
    }
    return d;
}

void UnderwaterNavModel::inObsRegion(UnderwaterNavState &sVals, Observation &obs) {
    obs.resize(2);
    /*
     if (cellType[sVals[0]][sVals[1]] == 3) {
     obs[0] = sVals[0]; obs[1] = sVals[1];
     }
     else {
     obs[0] = -1; obs[1] = -1;
     }
     */
    std::map<long, std::map<long, short> >::iterator itCellType = cellType_.find(
                sVals[0]);

    if (itCellType != cellType_.end()) {
        if (itCellType->second.find(sVals[1]) != itCellType->second.end()) {
            if (cellType_[sVals[0]][sVals[1]] == 3) {
                obs[0] = sVals[0];
                obs[1] = sVals[1];
                return;
            }
        }
    }
    obs[0] = -1;
    obs[1] = -1;
}

bool UnderwaterNavModel::inGoal(UnderwaterNavState &st) {
    //return cellType[st[0]][st[1]] == 1 ? true : false;
    for (UnderwaterNavState &goal : goals_) {
        if (st[0] == goal[0] && st[1] == goal[1]) {
            return true;
        }
    }
    return false;
}

bool UnderwaterNavModel::inRock(UnderwaterNavState &st) {
    //return cellType[st[0]][st[1]] == 2 ? true : false;
    for (UnderwaterNavState &rock : rocks_) {
        if (st[0] == rock[0] && st[1] == rock[1]) {
            return true;
        }
    }
    return false;

}
/*
 bool UnderwaterNavModel::inSpcRew(State &st) {
 std::vector<State>::iterator it;
 for (it = spcRew.begin(); it != spcRew.end(); it++) {
 if (st[0] == (*it)[0] && st[1] == (*it)[1]) {
 return true;
 }
 }
 return false;
 }
 */
double UnderwaterNavModel::getReward(UnderwaterNavState &sVals) {
    double totRew = 0.0;
    short ct = 0;

    std::map<long, std::map<long, short> >::iterator itCellType = cellType_.find(
                sVals[0]);

    if (itCellType != cellType_.end()) {
        if (itCellType->second.find(sVals[1]) != itCellType->second.end()) {
            ct = cellType_[sVals[0]][sVals[1]];
        }
    }
//cout << "st: " << sVals[0] << " " << sVals[1] << " ct " << ct << endl;
    switch (ct) {
    case 0: {
        totRew = 0.0;
        break;
    }
    case 1: {
        totRew = goalReward_;
        break;
    }
    case 2: {
        totRew = crashPenalty_;
        break;
    }
    case 3: {
        totRew = 0.0;
        break;
    }
        //default: { totRew = spcRew[ct-3]; break; }
    }
    return totRew;
}

double UnderwaterNavModel::getReward(UnderwaterNavState &sVals, unsigned long actId) {
    double totRew = 0.0;
    short ct = 0;

    std::map<long, std::map<long, short> >::iterator itCellType = cellType_.find(
                sVals[0]);

    if (itCellType != cellType_.end()) {
        if (itCellType->second.find(sVals[1]) != itCellType->second.end()) {
            ct = cellType_[sVals[0]][sVals[1]];
        }
    }
//cerr << "sVals: " << sVals[0] << " " << sVals[1] << " " << ct << endl;
    switch (ct) {
    case 0: {
        totRew = 0.0;
        break;
    }
    case 1: {
        totRew = goalReward_;
        break;
    }
    case 2: {
        totRew = crashPenalty_;
        break;
    }
    case 3: {
        totRew = 0.0;
        break;
    }
        //default: { totRew = spcRew[ct-3]; break; }
    }

    if (actId == EAST || actId == NORTH || actId == SOUTH) {
//cerr << "Straight action\n";
        totRew = totRew + moveCost_;
    } else {
//cerr << "diagonal action]n";
        totRew = totRew + moveDiagCost_;
    }
//cerr << "totRew: " << totRew << endl;
    return totRew;
}

bool UnderwaterNavModel::getNextState(UnderwaterNavState &sVals, unsigned long actId,
        double *immediateRew, UnderwaterNavState &nxtSVals, Observation &obs) {
    getNextState(sVals, actId, nxtSVals);
    inObsRegion(nxtSVals, obs);
//cerr << "GettingNxtState from " << sVals[0] << " " << sVals[1] << " act " << actId << endl;
    *immediateRew = getReward(sVals, actId);
//cerr << "imRew: " << *immediateRew << endl;
    return inGoal(nxtSVals);
}

void UnderwaterNavModel::getChangeTimes(char const *chName,
        std::vector<long> &chTime) {
    std::ifstream inFile;

    inFile.open(chName);
    if (!inFile.is_open()) {
        cerr << "Fail to open " << chName << "\n";
        std::exit(1);
    }

    chTime.clear();
    long t;
    long nChanges, nPrimChanges;
    string usrStr, tmpStr;
    inFile >> usrStr >> nChanges;
    for (long i = 0; i < nChanges; i++) {
        std::vector<string> aChange;
        inFile >> usrStr >> t >> usrStr >> nPrimChanges;
        chTime.push_back(t);
        std::getline(inFile, usrStr);
        for (long j = 0; j < nPrimChanges; j++) {
            std::getline(inFile, tmpStr);
            aChange.push_back(tmpStr);
        }
        changes_[t] = aChange;
    }
    inFile.close();
}

void UnderwaterNavModel::update(long tCh,
                                     std::vector<UnderwaterNavState> &affectedRange,
                                     std::vector<ChangeType> &typeOfChanges) {
    affectedRange.clear();
    std::vector<UnderwaterNavState>::iterator itStVals;
    string cmd, var, tmpStr;
    for (string changeString : changes_[tCh]) {
        std::stringstream ss(changeString);
        ss >> cmd;
        if (cmd.compare("Add") == 0) {
            ss >> var;
            if (var.compare("Rocks") == 0) {
                double xl, yl, xu, yu;
                ss >> tmpStr >> xl >> tmpStr >> yl >> tmpStr >> tmpStr >> xu
                   >> tmpStr >> yu >> tmpStr;
                UnderwaterNavState sl;
                sl.push_back(xl);
                sl.push_back(yl);
                UnderwaterNavState su;
                su.push_back(xu);
                su.push_back(yu);
                affectedRange.push_back(sl);
                affectedRange.push_back(su);
                typeOfChanges.push_back(ChangeType::REWARD);
                long x = (long) std::floor(xl);
                while (x < xu) {
                    long y = (long) std::floor(yl);
                    while (y < yu) {
                        if (cellType_[x][y] == 1) {
                            itStVals = getIterator(goals_, x, y);
                            if (itStVals != goals_.end()) {
                                goals_.erase(itStVals);
                            }
                        } else if (cellType_[x][y] == 3) {
                            itStVals = getIterator(allObservations_, x, y);
                            if (itStVals != allObservations_.end()) {
                                allObservations_.erase(itStVals);
                            }
                        }
                        cellType_[x][y] = 2;
                        UnderwaterNavState s(2);
                        s[0] = x;
                        s[1] = y;
//cerr << "AddRocks ( " << s[0] << " " << s[1] << " ) ";
                        rocks_.push_back(s);
                        nRocks_++;
                        y = y + 1;
                    }
                    x = x + 1;
                }
            } else if (var.compare("Obstacles") == 0) {
                double xl, yl, xu, yu;
                ss >> tmpStr >> xl >> tmpStr >> yl >> tmpStr >> tmpStr >> xu
                   >> tmpStr >> yu >> tmpStr;
                UnderwaterNavState sl;
                sl.push_back(xl);
                sl.push_back(yl);
                UnderwaterNavState su;
                su.push_back(xu);
                su.push_back(yu);
                obstacleRegion_.push_back(sl);
                obstacleRegion_.push_back(su);
                affectedRange.push_back(sl);
                affectedRange.push_back(su);
                typeOfChanges.push_back(ChangeType::ADDOBSTACLE);
                long x = (long) std::floor(xl);
                while (x < xu) {
                    long y = (long) std::floor(yl);
                    while (y < yu) {
                        if (cellType_[x][y] == 1) {
                            itStVals = getIterator(goals_, x, y);
                            if (itStVals != goals_.end()) {
                                goals_.erase(itStVals);
                            }
                        } else if (cellType_[x][y] == 2) {
                            itStVals = getIterator(rocks_, x, y);
                            if (itStVals != rocks_.end()) {
                                rocks_.erase(itStVals);
                            }
                        }
                        cellType_[x][y] = 5;
                        y = y + 1;
                    }
                    x = x + 1;
                }
            } else if (var.compare("Observations") == 0) {
                double xl, yl, xu, yu;
                ss >> tmpStr >> xl >> tmpStr >> yl >> tmpStr >> tmpStr >> xu
                   >> tmpStr >> yu >> tmpStr;
                UnderwaterNavState sl;
                sl.push_back(xl);
                sl.push_back(yl);
                UnderwaterNavState su;
                su.push_back(xu);
                su.push_back(yu);
                affectedRange.push_back(sl);
                affectedRange.push_back(su);
                typeOfChanges.push_back(ChangeType::ADDOBSERVATION);
                long x = (long) std::floor(xl);
                while (x < xu) {
                    long y = (long) std::floor(yl);
                    while (y < yu) {
                        if (cellType_[x][y] == 1) {
                            itStVals = getIterator(goals_, x, y);
                            if (itStVals != goals_.end()) {
                                goals_.erase(itStVals);
                            }
                        } else if (cellType_[x][y] == 2) {
                            itStVals = getIterator(rocks_, x, y);
                            if (itStVals != rocks_.end()) {
                                rocks_.erase(itStVals);
                            }
                        }
                        cellType_[x][y] = 3;
                        UnderwaterNavState s(2);
                        s[0] = x;
                        s[1] = y;
                        allObservations_.push_back(s);
                        nObservations_++;
                        y = y + 1;
                    }
                    x = x + 1;
                }

            }

            else {
                cerr << "Invalid update: " << cmd << " " << var << ".\n";
            }
        } else if (cmd.compare("Del") == 0) {
            ss >> var;
            if (var.compare("Rocks") == 0) {
                double xl, yl, xu, yu;
                ss >> tmpStr >> xl >> tmpStr >> yl >> tmpStr >> tmpStr >> xu
                   >> tmpStr >> yu >> tmpStr;
                UnderwaterNavState sl;
                sl.push_back(xl);
                sl.push_back(yl);
                UnderwaterNavState su;
                su.push_back(xu);
                su.push_back(yu);
                affectedRange.push_back(sl);
                affectedRange.push_back(su);
                typeOfChanges.push_back(ChangeType::REWARD);
                std::vector<UnderwaterNavState>::iterator itSt;
                long x = (long) std::floor(xl);
                while (x < xu) {
                    long y = (long) std::floor(yl);
                    while (y < yu) {
                        cellType_[x][y] = 0;
                        nRocks_--;
                        y = y + 1;
                    }
                    x = x + 1;
                }
            } else {
                cerr << "Invalid update: " << cmd << " " << var << ".\n";
            }
        } else if (cmd.compare("Change")) {
            ss >> var;
            if (var.compare("Rewards") == 0) {
                double xl, yl, xu, yu;
                ss >> tmpStr >> xl >> tmpStr >> yl >> tmpStr >> tmpStr >> xu
                   >> tmpStr >> yu >> tmpStr;
                UnderwaterNavState sl;
                sl.push_back(xl);
                sl.push_back(yl);
                UnderwaterNavState su;
                su.push_back(xu);
                su.push_back(yu);
                affectedRange.push_back(sl);
                affectedRange.push_back(su);
                typeOfChanges.push_back(ChangeType::REWARD);
                double newRew;
                ss >> tmpStr >> newRew;
                spcRew_.push_back(newRew);
                long x = (long) std::floor(xl);
                while (x < xu) {
                    long y = (long) std::floor(yl);
                    while (y < yu) {
                        cellType_[x][y] = nSpcRew_;
                        y = y + 1;
                    }
                    x = x + 1;
                }
                nSpcRew_++;
            }
            /*
             else if (var.compare("Transition") == 0) {
             double xl, yl, xu, yu;
             ss >> tmpStr >> xl >> tmpStr >> yl >> tmpStr >> tmpStr >> xu >> tmpStr >> yu >> tmpStr;
             State sl; sl.push_back(xl); sl.push_back(yl);
             State su; su.push_back(xl); su.push_back(yl);
             affectedRange.push_back(sl); affectedRange.push_back(su);
             typesOfChanges.push_back(ChangeType::TRANSITION);
             //...
             }
             */
            else {
                cerr << "Invalid update: " << cmd << " " << var << ".\n";
            }
        } else {
            cerr << "Invalid command: " << cmd << "\n";
        }
    }
    delete roadmap_;
    roadmap_ = new StRoadmap(randGen_, goals_, nVerts_, nGoals_, nTryCon_, maxDistCon_,
                            cellType_, nX_, nY_);
    //roadmap->updateRoadmap(cellType, goals, nGoals);

//cerr << "#affected: " << affectedRange.size() << " #type: " << typeOfChanges.size() << endl;
    /*
     std::vector<State>::iterator itR;
     long z = 0;
     for (itR = rocks.begin(); itR != rocks.end(); itR++, z++) {
     cerr << "Rock-" << z << " ( " << (*itR)[0] << " " << (*itR)[1] << endl;
     }
     */
}

double UnderwaterNavModel::getDefaultVal() {
    return minVal_;
}

std::vector<UnderwaterNavState>::iterator UnderwaterNavModel::getIterator(
    std::vector<UnderwaterNavState> &vecStVals, long x, long y) {
    std::vector<UnderwaterNavState>::iterator it;
    for (it = vecStVals.begin(); it != vecStVals.end(); it++) {
        if ((*it)[0] == x && (*it)[1] == y) {
            return it;
        }
    }
    return vecStVals.end();
}

void UnderwaterNavModel::getReachableSt(UnderwaterNavState &s, unsigned long actId,
        std::vector<UnderwaterNavState> &nxtS) {
    nxtS.clear();
    switch (actId) {
    case EAST: {
        UnderwaterNavState sp(2);
        sp[0] = s[0] + 1.0;
        sp[1] = s[1];
        nxtS.push_back(sp);
        sp[0] = s[0] + 1.0;
        sp[1] = s[1] + 1.0;
        nxtS.push_back(sp);
        sp[0] = s[0] + 1.0;
        sp[1] = s[1] - 1.0;
        nxtS.push_back(sp);
        break;
    }
    case NORTH: {
        UnderwaterNavState sp(2);
        sp[0] = s[0];
        sp[1] = s[1] - 1.0;
        nxtS.push_back(sp);
        sp[0] = s[0] - 1.0;
        sp[1] = s[1] - 1.0;
        nxtS.push_back(sp);
        sp[0] = s[0] + 1.0;
        sp[1] = s[1] - 1.0;
        nxtS.push_back(sp);
        break;
    }
    case SOUTH: {
        UnderwaterNavState sp(2);
        sp[0] = s[0];
        sp[1] = s[1] + 1.0;
        nxtS.push_back(sp);
        sp[0] = s[0] - 1.0;
        sp[1] = s[1] + 1.0;
        nxtS.push_back(sp);
        sp[0] = s[0] + 1.0;
        sp[1] = s[1] + 1.0;
        nxtS.push_back(sp);
        break;
    }
    case NORTHEAST: {
        UnderwaterNavState sp(2);
        sp[0] = s[0] + 1.0;
        sp[1] = s[1] - 1.0;
        nxtS.push_back(sp);
        sp[0] = s[0];
        sp[1] = s[1] - 1.0;
        nxtS.push_back(sp);
        sp[0] = s[0] + 1.0;
        sp[1] = s[1];
        nxtS.push_back(sp);
        break;
    }
    case SOUTHEAST: {
        UnderwaterNavState sp(2);
        sp[0] = s[0] + 1.0;
        sp[1] = s[1] + 1.0;
        nxtS.push_back(sp);
        sp[0] = s[0];
        sp[1] = s[1] + 1.0;
        nxtS.push_back(sp);
        sp[0] = s[0] + 1.0;
        sp[1] = s[1];
        nxtS.push_back(sp);
        break;
    }
    }
}

void UnderwaterNavModel::getStatesSeeObs(unsigned long actId,
        Observation &obs, std::vector<UnderwaterNavState> &partSt,
        std::vector<UnderwaterNavState> &partNxtSt) {
    for (UnderwaterNavState &sVals : partSt) {
        std::vector<UnderwaterNavState> reachableSt;
        getReachableSt(sVals, actId, reachableSt);
        for (UnderwaterNavState &nxtSt : reachableSt) {
            if (nxtSt[0] == obs[0] && nxtSt[1] == obs[1]) {
                partNxtSt.push_back(nxtSt);
                break;
            }
        }
    }
}

void UnderwaterNavModel::getStatesSeeObs(unsigned long /*actId*/,
        Observation &obs, std::vector<UnderwaterNavState> &partNxtSt) {
    UnderwaterNavState s(2);
    s[0] = obs[0];
    s[1] = obs[1];
    partNxtSt.push_back(s);
}

int UnderwaterNavModel::findCollision(UnderwaterNavState &s) {
    int i;
    std::vector<UnderwaterNavState>::iterator it1 = obstacleRegion_.begin();
    std::vector<UnderwaterNavState>::iterator it2 = obstacleRegion_.begin() + 1;
    /*
     cerr << "obsSize: " << obstacleRegion.size() << endl;
     for (i = 0; it1 != obstacleRegion.end(); i++) {
     cerr << " ( " << s[0] << " " << s[1] << " ) Obs " << (*it1)[0] << " " << (*it1)[1] << " to " << (*it2)[0] << " " << (*it2)[1] << endl;
     it1++; it1++; it2++; it2++;
     }
     */
    for (i = 0; it1 != obstacleRegion_.end(); i++) {
//cerr << " ( " << s[0] << " " << s[1] << " ) Obs " << (*it1)[0] << " " << (*it1)[1] << " to " << (*it2)[0] << " " << (*it2)[1] << endl;
        if ((*it1)[0] <= s[0] && s[0] < (*it2)[0] && (*it1)[1] <= s[1]
                && s[1] < (*it2)[1]) {
            return i;
        }
        it1++;
        it1++;
        it2++;
        it2++;
    }
//cerr << endl;
    return -1;
}

bool UnderwaterNavModel::modifStSeq(std::vector<UnderwaterNavState> &seqStVals,
        long startAffectedIdx, long endAffectedIdx,
        std::vector<UnderwaterNavState> &modifStSeq, std::vector<long> &modifActSeq,
        std::vector<Observation> &modifObsSeq,
        std::vector<double> &modifRewSeq) {

    // Get nearest vertex of affected region.
    long startAffected = startAffectedIdx;
    long affByIdx = -1;
    while (startAffected <= endAffectedIdx && affByIdx == -1) {
        affByIdx = findCollision(seqStVals[startAffected]);
        startAffected++;
    }

    if (affByIdx == -1) {
        return false;
    }

//cerr << "AffByIdx " << affByIdx << " " << startAffected << " " << startAffectedIdx << " affPos " << seqStVals[startAffected-1][0]
//      << " " << seqStVals[startAffected-1][1] << endl;
    startAffectedIdx = startAffected - 1;

    std::vector<UnderwaterNavState> affRegV;
    affRegV.push_back(obstacleRegion_[affByIdx]);        // ll
    affRegV.push_back(obstacleRegion_[affByIdx + 1]);      // ru
    UnderwaterNavState s(2);
    s[0] = obstacleRegion_[2 * affByIdx][0];
    s[1] = obstacleRegion_[2 * affByIdx + 1][1];
    affRegV.push_back(s);
    s[0] = obstacleRegion_[2 * affByIdx + 1][0];
    s[1] = obstacleRegion_[2 * affByIdx][1];
    affRegV.push_back(s);
    s = seqStVals[startAffectedIdx];
    double tmpDist;
    double minDist = LONG_MAX;
    short minIdx = -1;
    short i = 0;
    for (UnderwaterNavState &sv : affRegV) {
        if (sv[0] > 0 && sv[1] > 0 && sv[0] < nX_ - 1 && sv[1] < nY_ - 1
                && (tmpDist = getDist(s, sv)) < minDist) {
            minDist = tmpDist;
            minIdx = i;
        }
    }
//cerr << "minDist: " << minDist << " " << minIdx << " V " << affRegV[minIdx][0] << " " << affRegV[minIdx][1] << endl;
    UnderwaterNavState subGoal(2);
    switch (minIdx) {
    case 0: {   // lower left
        subGoal[0] = affRegV[minIdx][0] - 1;
        subGoal[1] = affRegV[minIdx][1] - 1;
        break;
    }
    case 1: {   // upper right
        subGoal[0] = affRegV[minIdx][0];
        subGoal[1] = affRegV[minIdx][1];
        break;
    }
    case 2: {
        subGoal[0] = affRegV[minIdx][0] - 1;
        subGoal[1] = affRegV[minIdx][1];
        break;
    }
    case 3: {
        subGoal[0] = affRegV[minIdx][0];
        subGoal[1] = affRegV[minIdx][1] - 1;
        break;
    }
    }

//cerr << "ok2 subGoal: " << subGoal[0] << " " << subGoal[1] << "\n";
    // Get sequence of actions to move from startAffectedIdx-1 to the nearest vertex of affected region.
    long tmpAct;
    long nAct = 0;
    UnderwaterNavState sStart;
    if (startAffectedIdx > 0) {
        sStart = seqStVals[startAffectedIdx - 1];
        if (getDist(sStart, subGoal) == 0) {
            nAct = 0;
        } else if (sStart[0] == subGoal[0]) {
            if (sStart[1] < subGoal[1]) {
                tmpAct = SOUTH;
                nAct = subGoal[1] - sStart[1];
            } else {
                tmpAct = NORTH;
                nAct = sStart[1] - subGoal[1];
            }
        } else if (sStart[1] == subGoal[1]) {
            if (sStart[0] < subGoal[0]) {
                tmpAct = EAST;
                nAct = subGoal[0] - sStart[0];
            } else {
                subGoal[0] = sStart[0];
                nAct = 0;
            }
        }
    } else {
//cerr << "oout1\n";
        return false;
    }
    modifStSeq.push_back(sStart);
    UnderwaterNavState prevS = sStart;
    for (long i = 0; i < nAct; i++) {
        modifActSeq.push_back(tmpAct);
        switch (tmpAct) {
        case EAST: {
            s[0] = prevS[0] + 1;
            s[1] = prevS[1];
            break;
        }
        case NORTH: {
            s[0] = prevS[0];
            s[1] = prevS[1] - 1;
            break;
        }
        case SOUTH: {
            s[0] = prevS[0];
            s[1] = prevS[1] + 1;
            break;
        }
        case NORTHEAST: {
            s[0] = prevS[0] + 1;
            s[1] = prevS[1] - 1;
            break;
        }
        case SOUTHEAST: {
            s[0] = prevS[0] + 1;
            s[1] = prevS[1] + 1;
            break;
        }
        }
        modifStSeq.push_back(s);
        prevS = s;
    }
    /*
     cerr << "ok3\n";
     std::vector<State>::iterator it1;
     for (it1 = modifStSeq.begin(); it1 != modifStSeq.end(); it1++) {
     cerr << "new " << (*it1)[0] << " " << (*it1)[1] << endl;
     }
     cerr << "ok3ok3\n";
     */
    // Fix the rest of the path.
    if (endAffectedIdx >= 0) {
        UnderwaterNavState endS = seqStVals[endAffectedIdx + 1];
        sStart = prevS;
        if (getDist(sStart, endS) == 0) {
        } else if (sStart[0] > endS[0]) {
        } else {
            switch (minIdx) {
            case 0:
            case 3: {
                if (sStart[1] != endS[1]) {
                    for (long i = (long) sStart[0] + 1; i <= (long) endS[0];
                            i++) {
                        s[0] = i;
                        s[1] = sStart[1];
                        modifStSeq.push_back(s);
                        modifActSeq.push_back(EAST);
                    }
                    sStart = s;
                    for (long i = (long) sStart[1] + 1; i < (long) endS[1];
                            i++) {
                        s[0] = endS[0];
                        s[1] = i;
                        modifStSeq.push_back(s);
                        modifActSeq.push_back(SOUTH);
                    }
                    modifActSeq.push_back(SOUTH);
                } else {
                    for (long i = (long) sStart[0] + 1; i < (long) endS[0];
                            i++) {
                        s[0] = i;
                        s[1] = sStart[1];
                        modifStSeq.push_back(s);
                        modifActSeq.push_back(EAST);
                    }
                    modifActSeq.push_back(EAST);
                }
                break;
            }
            case 1:
            case 2: {
                if (sStart[1] != endS[1]) {
                    for (long i = (long) sStart[0] + 1; i <= (long) endS[0];
                            i++) {
                        s[0] = i;
                        s[1] = sStart[1];
                        modifStSeq.push_back(s);
                        modifActSeq.push_back(EAST);
                    }
                    sStart = s;
                    for (long i = (long) sStart[1] - 1; i > (long) endS[1];
                            i--) {
                        s[0] = endS[0];
                        s[1] = i;
                        modifStSeq.push_back(s);
                        modifActSeq.push_back(NORTH);
                    }
                    modifActSeq.push_back(NORTH);
                } else {
                    for (long i = (long) sStart[0] + 1; i < (long) endS[0];
                            i++) {
                        s[0] = i;
                        s[1] = sStart[1];
                        modifStSeq.push_back(s);
                        modifActSeq.push_back(EAST);
                    }
                    modifActSeq.push_back(EAST);
                }
                break;
            }
            }
        }
    }
    /*
     cerr << "ok4\n";
     for (it1 = modifStSeq.begin(); it1 != modifStSeq.end(); it1++) {
     cerr << "new " << (*it1)[0] << " " << (*it1)[1] << endl;
     }
     cerr << "ok4ok4\n";
     */
    // Set observation and reward.
    std::vector<UnderwaterNavState>::iterator it;
    std::vector<long>::iterator itAct;
    for (it = modifStSeq.begin(), itAct = modifActSeq.begin();
            itAct != modifActSeq.end(); it++, itAct++) {
        modifRewSeq.push_back(getReward(*it, *itAct));
        Observation obs;
        inObsRegion(*it, obs);
        modifObsSeq.push_back(obs);
    }
    if (it != modifStSeq.end()) {
        modifRewSeq.push_back(getReward(*it));
        Observation obs;
        inObsRegion(*it, obs);
        modifObsSeq.push_back(obs);
    }

    return true;
}

bool UnderwaterNavModel::isTerm(UnderwaterNavState &sVals) {
    short ct = 0;
    std::map<long, std::map<long, short> >::iterator itCellType = cellType_.find(
                sVals[0]);
    if (itCellType != cellType_.end()) {
        if (itCellType->second.find(sVals[1]) != itCellType->second.end()) {
            ct = cellType_[sVals[0]][sVals[1]];
        }
    }
    return (ct == 1) ? true : false;
}

void UnderwaterNavModel::dispAct(Action const &action, std::ostream &os) {
        switch (action) {
        case EAST:
            os << "EAST";
            break;
        case NORTH:
            os << "NORTH";
            break;
        case SOUTH:
            os << "SOUTH";
            break;
        case NORTHEAST:
            os << "NORTHEAST";
            break;
        case SOUTHEAST:
            os << "SOUTHEAST";
            break;
        default:
            os << "ERROR-" << actId;
            break;
        }
    }

    void UnderwaterNavModel::dispObs(Observation const &obs, std::ostream &os) {
        if (obs[0] == -1 && obs[1] == -1) {
            os << "NONE";
            return;
        }
        os << "(" << obs[0] << ", " << obs[1] << ")";
    }

void UnderwaterNavModel::drawEnv(std::ostream &os) {
    std::map<long, std::map<long, short> >::iterator itCellType;
    os << endl;
    for (unsigned long y = 0; y < nY_; y++) {
        for (unsigned long x = 0; x < nX_; x++) {
            itCellType = cellType_.find(x);
            if (itCellType != cellType_.end()) {
                if (itCellType->second.find(y) != itCellType->second.end()) {
                    // 0: usual, 1: goals, 2: rocks, 3: observation, 4: spc. reward, 5: obstacle.
                    switch (cellType_[x][y]) {
                    case 1: {
                        os << "D";
                        break;
                    }
                    case 2: {
                        os << "R";
                        break;
                    }
                    case 3: {
                        os << "O";
                        break;
                    }
                    case 5: {
                        os << "W";
                        break;
                    }
                    default: {
                        os << cellType_[x][y];
                        break;
                    }
                    }
                } else {
                    os << " ";
                }
            } else {
                os << " ";
            }
        }
        os << " " << endl;
    }
}

void UnderwaterNavModel::drawState(UnderwaterNavState &s, std::ostream &os) {
    std::map<long, std::map<long, short> >::iterator itCellType;
    os << endl;
    for (unsigned long y = 0; y < nY_; y++) {
        for (unsigned long x = 0; x < nX_; x++) {
            if (x == s[0] && y == s[1]) {
                os << "X";
                continue;
            }
            itCellType = cellType_.find(x);
            if (itCellType != cellType_.end()) {
                if (itCellType->second.find(y) != itCellType->second.end()) {
                    // 0: usual, 1: goals, 2: rocks, 3: observation, 4: spc. reward, 5: obstacle.
                    switch (cellType_[x][y]) {
                    case 1: {
                        os << "D";
                        break;
                    }
                    case 2: {
                        os << "R";
                        break;
                    }
                    case 3: {
                        os << "O";
                        break;
                    }
                    case 5: {
                        os << "W";
                        break;
                    }
                    default: {
                        os << cellType_[x][y];
                        break;
                    }
                    }
                } else {
                    os << " ";
                }
            } else {
                os << " ";
            }
        }
        os << " " << endl;
    }
}
} /* namespace uwnav */
