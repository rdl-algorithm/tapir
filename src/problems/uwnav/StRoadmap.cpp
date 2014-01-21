#include "StRoadmap.hpp"

#include <climits>                      // for LONG_MAX
#include <cmath>                        // for fabs, floor
#include <map>                          // for map, _Rb_tree_iterator, map<>::iterator, map<>::mapped_type, multimap, multimap<>::iterator, __alloc_traits<>::value_type
#include <ostream>                      // for operator<<, ostream, endl, basic_ostream::operator<<, basic_ostream
#include <utility>                      // for pair, make_pair
#include <vector>                       // for vector, vector<>::iterator, vector<>::reference

#include "defs.hpp"                     // for RandomGenerator
#include "solver/State.hpp"                    // for State
#include "uwnav.hpp"
#include "UnderwaterNavState.hpp"

using std::endl;

namespace uwnav {
#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Weffc++"
StRoadmap::StRoadmap(RandomGenerator *randGen,
        std::vector<UnderwaterNavState> &goals, long maxVerts, long nGoalsSamp,
        long nTryCon, long maxDistCon,
        std::map<long, std::map<long, UnderwaterNavCellType> > &env, long nCols,
        long nRows) :
        randGen_(randGen),
        nCols_(nCols),
        nRows_(nRows),
        env_(env),
        weight_(),
        totW_(0),
        maxTryCon_(nTryCon),
        maxDistCon_(maxDistCon),
        nVerts_(0),
        maxVerts_(maxVerts),
        lastGoalIdx_(0),
        milestones_(),
        outEdges_(),
        inEdges_(),
        shortestDistToGoal_() {
#pragma GCC diagnostic pop
    setWeight();
    insertGoalMilestones(goals, nGoalsSamp);
    long i = lastGoalIdx_;

    //insertMyMilestones();

    while (i < maxVerts) {
        UnderwaterNavState s = sampleAMilestone();
        if (insertMilestone(s)) {
            i++;
        }
    }
    /*
     std::vector<State>::iterator itV;
     cerr << "Vertices: (lastGoal "  << lastGoalIdx << ")\n";
     i = 0;
     for (itV = V.begin(); itV != V.end(); itV++, i++) {
     cerr << "v-" << i << " " << (*itV)[0] << " " << (*itV)[1] << endl;
     }
     cerr << "Edges:\n";
     std::map< long, std::vector< std::pair<long, long> > >::iterator it1;
     std::vector< std::pair<long, long> >::iterator it2;
     for (it1 = outEdges.begin(); it1 != outEdges.end(); it1++) {
     cerr << "From " << it1->first;
     for (it2 = it1->second.begin(); it2 != it1->second.end(); it2++) {
     cerr << " to " << it2->first;
     // << " , c: " << it2->second << " ";
     }
     cerr << endl;
     }
     */
    getDistToGoal();
    /*
     std::map<long, long>::iterator itM;
     for (itM = shortestDistToGoal.begin(); itM != shortestDistToGoal.end(); itM++) {
     cerr << "distToGoal From " << itM->first << " cost " << itM->second << endl;
     }
     */
//  draw(cerr);
}

void StRoadmap::setWeight() {
    std::map<long, std::map<long, UnderwaterNavCellType> >::iterator itCol;
    std::map<long, UnderwaterNavCellType>::iterator itRow;
    totW_ = 0;
    long prevCol = 0;
    long prevRow, pivIdx;
    for (itCol = env_.begin(); itCol != env_.end(); itCol++) {
        for (long col = prevCol; col < itCol->first; col++) {
//cerr << "Fr " << prevX << " to " << itX->first << " curr " << x
//      << " prevY " << prevY << " to " << itY->first << endl;
            pivIdx = col * nRows_;
            for (long row = 0; row < nRows_; row++) {
                totW_++;
                weight_[totW_] = pivIdx + row;
//cerr << "LeftOverY weight[" << totW << "] : " << x << " " << y << " " << weight[totW] << endl;
            }
        }
//cerr << "At " << itX->first << endl;
        prevRow = 0;
        pivIdx = itCol->first * nRows_;
        for (itRow = itCol->second.begin(); itRow != itCol->second.end();
                itRow++) {
            for (long y = prevRow; y < itRow->first; y++) {
                totW_++;
                weight_[totW_] = pivIdx + y;
//cerr << "weight[" << totW << "] : " << itX->first << " " << y << " " << weight[totW] << endl;
            }
            if (itRow->second == UnderwaterNavCellType::OBSERVATION) { // Observation
//cerr << "OBS ( " << itX->first << " " << itY->first << " ) ";
                totW_++;
                weight_[totW_] = pivIdx + itRow->first;
//cerr << "weight[" << totW << "] : " << itX->first << " " << itY->first << " " << weight[totW] << " ";
                totW_++;
                weight_[totW_] = pivIdx + itRow->first;
//cerr << "weight[" << totW << "] : " << itX->first << " " << itY->first << " " << weight[totW] << endl;
            }
            /*
             else if (itY->second == 0) { // Usual empty space
             totW++;
             weight[totW] = pivIdx + itY->first;
             }
             */
            /*
             if (itY->second == 2) {
             cerr << "ROCKS ( " << itX->first << " " << itY->first << " ) ";
             }
             */
            prevRow = itRow->first + 1;
        }
        for (long y = prevRow; y < nRows_; y++) {
            totW_++;
            weight_[totW_] = pivIdx + y;
//cerr << "LeftOverY weight[" << totW << "] : " << itX->first << " " << y << " " << weight[totW] << endl;
        }

        prevCol = itCol->first + 1;
    }
    for (long x = 0; x < env_.begin()->first; x++) {
        pivIdx = x * nRows_;
        for (long y = 0; y < nRows_; y++) {
            totW_++;
            weight_[totW_] = pivIdx + y;
//cerr << "LeftOverX weight[" << totW << "] : " << x << " " << y << " " << weight[totW] << endl;
        }
    }
    for (long x = prevCol; x < nCols_; x++) {
        pivIdx = x * nRows_;
        for (long y = 0; y < nRows_; y++) {
            totW_++;
            weight_[totW_] = pivIdx + y;
//cerr << "LeftOverX weight[" << totW << "] : " << x << " " << y << " " << weight[totW] << endl;
        }
    }
}

void StRoadmap::insertGoalMilestones(std::vector<UnderwaterNavState> &goals,
        long nGoalsSamp) {
    long nGoals = goals.size();
    //for (long i = 0; i < nGoalsSamp; i++) {
    std::vector<UnderwaterNavState>::iterator itV;
    bool inserted;
    while (nVerts_ < nGoalsSamp) {
        long rawIdx = std::uniform_int_distribution<long>(0, nGoals - 1)(
                *randGen_);
        UnderwaterNavState state = goals[rawIdx];
        inserted = true;
        for (itV = milestones_.begin(); itV != milestones_.end(); itV++) {
            if (dist(state, *itV) == 0) {
                inserted = false;
                break;
            }
        }
        if (inserted) {
            milestones_.push_back(state);
            nVerts_++;
        }
    }
    lastGoalIdx_ = nVerts_ - 1;
}

void StRoadmap::insertMyMilestones() {
    std::vector<UnderwaterNavState> s;

    s.emplace_back(20, 14);
    s.emplace_back(40, 14);
    s.emplace_back(20, 24);
    s.emplace_back(40, 24);
    s.emplace_back(18, 33);
    s.emplace_back(18, 41);

    std::vector<UnderwaterNavState>::iterator it;
    for (it = s.begin(); it != s.end(); it++) {
        insertMilestone(*it);
    }
}

UnderwaterNavState StRoadmap::sampleAMilestone() {
    long rawIdx = std::uniform_int_distribution<long>(0, totW_)(*randGen_);
    /*
     if (st[0] >= 35 && st[0] < 39 && st[1] >= 20 && st[1] < 52) {
     cerr << "SAMPLE ROCK raw " << rawIdx << " w " << weight[rawIdx] << " env "
     << env[st[0]][st[1]] << " ( " << st[0] << " " << st[1] << " ) "
     << weight[rawIdx-1] << " " << weight[rawIdx+1] << endl;
     }
     */
    return UnderwaterNavState(weight_[rawIdx] % nRows_,
            (long) std::floor(weight_[rawIdx] / nRows_));
}

bool StRoadmap::insertMilestone(UnderwaterNavState state) {
    long c, i;
    std::vector<UnderwaterNavState>::iterator itV;
    for (itV = milestones_.begin(), i = 0; itV != milestones_.end();
            itV++, i++) {
        if ((c = dist(state, *itV)) == 0) {
            return false;
        }
    }

    long nTry = 0;
    for (itV = milestones_.begin(), i = 0; itV != milestones_.end();
            itV++, i++) {
        if (dist(state, *itV) < maxDistCon_ && nTry < maxTryCon_ && (c =
                lineSegOk(state, *itV)) > -1) {
            outEdges_[nVerts_].push_back(std::make_pair(i, c));
            inEdges_[i].push_back(std::make_pair(nVerts_, c));
        }
        nTry++;
    }
    milestones_.push_back(state);
    nVerts_++;
    return true;
}

double StRoadmap::dist(UnderwaterNavState &s1, UnderwaterNavState &s2) {
    return s1.distanceTo(s2);
}

long StRoadmap::lineSegOk(UnderwaterNavState &st1, UnderwaterNavState &st2) {
    std::map<long, std::map<long, UnderwaterNavCellType> >::iterator itX;
    std::map<long, UnderwaterNavCellType>::iterator itY;

    GridPosition p1 = st1.getPosition();
    GridPosition p2 = st2.getPosition();

    // Move until Y = end Y position, and then move in X direction.
    if ((itX = env_.find(p1.j)) == env_.end()) {
        for (long j = p1.j; j < p2.j; j++) {
            itX = env_.find(j);
            if (itX != env_.end()) {
                if ((itY = itX->second.find(p2.i)) != itX->second.end()) {
                    if (itY->second == UnderwaterNavCellType::ROCK
                            || itY->second == UnderwaterNavCellType::OBSTACLE) {
                        return -1;
                    }
                }
            }
        }
    } else {
        for (long y = st1[1]; y < st2[2]; y++) {
            if ((itY = itX->second.find(y)) != itX->second.end()) {
                if (itY->second == UnderwaterNavCellType::ROCK
                        || itY->second == UnderwaterNavCellType::OBSTACLE) {
                    return -1;
                }
            }
        }
        for (long x = st1[0]; x < st2[0]; x++) {
            itX = env_.find(x);
            if (itX != env_.end()) {
                if ((itY = itX->second.find(st2[1])) != itX->second.end()) {
                    if (itY->second == 2 || itY->second == 5) {
                        return -1;
                    }
                }
            }
        }
    }
    return dist(st1, st2);
}

// Dijkstra shortest path.
void StRoadmap::getDistToGoal() {
    std::vector<std::pair<long, long> >::iterator itNxt;
    long tmpDist;

    // Find shortest path to each goal
    std::vector<std::vector<long> > allDist;
    for (long i = 0; i <= lastGoalIdx_; i++) {
        std::vector<long> dist(nVerts_, LONG_MAX);
        std::vector<long> prev(nVerts_);
        std::vector<bool> visited(nVerts_, false);

        for (long j = 0; j <= lastGoalIdx_; j++) {
            dist[j] = 0;
            visited[j] = true;
        }

        // Iteration
        std::multimap<long, long> q;
        std::map<long, std::multimap<long, long>::iterator> ptrToIdx;
        for (int j = lastGoalIdx_ + 1; j < nVerts_; j++) {
            ptrToIdx[j] = q.insert(std::pair<long, long>(LONG_MAX, j));
        }
        for (int j = 0; j <= lastGoalIdx_; j++) {
            ptrToIdx[j] = q.end();
        }
        for (itNxt = inEdges_[i].begin(); itNxt != inEdges_[i].end(); itNxt++) {
//cerr << "About to erase " << itNxt->first << " of v " << i << endl;
            q.erase(ptrToIdx[itNxt->first]);
            ptrToIdx[itNxt->first] = q.insert(
                    std::pair<long, long>(itNxt->second, itNxt->first));
            dist[itNxt->first] = itNxt->second;
            //nxt[itNxt->first] = i;
        }
        long currIdx;
        std::multimap<long, long>::iterator itQ;
        while (!q.empty()) {
            itQ = q.begin();
            currIdx = itQ->second;
            for (itNxt = inEdges_[currIdx].begin();
                    itNxt != inEdges_[currIdx].end(); itNxt++) {
                if (!visited[itNxt->first]
                        && (tmpDist = itQ->first + itNxt->second)
                                < ptrToIdx[itNxt->first]->first) {
                    q.erase(ptrToIdx[itNxt->first]);
                    ptrToIdx[itNxt->first] = q.insert(
                            std::pair<long, long>(tmpDist, itNxt->first));
                    dist[itNxt->first] = tmpDist;
                    //nxt[itNxt->first] = currIdx;
                }
            }
            visited[currIdx] = true;
            q.erase(itQ);
        }
        /*
         std::vector<long>::iterator itD;
         cerr << "Dist ";
         for (itD = dist.begin(); itD != dist.end(); itD++) {
         cerr << *itD << " ";
         }
         cerr << endl;
         */
        allDist.push_back(dist);
    }

    std::vector<std::vector<long> >::iterator itVecVecL = allDist.begin();
    std::vector<long>::iterator itVecL;
    for (long i = 0; i < nVerts_; i++) {
        shortestDistToGoal_[i] = LONG_MAX;
    }
    for (itVecVecL->begin(); itVecVecL != allDist.end(); itVecVecL++) {
        long i = 0;
        for (itVecL = itVecVecL->begin(); itVecL != itVecVecL->end();
                itVecL++, i++) {
            if (*itVecL > 0 && *itVecL < shortestDistToGoal_[i]) {
                shortestDistToGoal_[i] = *itVecL;
            }
        }
    }

    for (long i = 0; i <= lastGoalIdx_; i++) {
        shortestDistToGoal_[i] = 0;
    }
    for (long i = lastGoalIdx_ + 1; i < nVerts_; i++) {
        if (shortestDistToGoal_[i] == LONG_MAX) { // For speed, we don't bother to erase the graph.
            shortestDistToGoal_.erase(i);
        }
    }
}

double StRoadmap::getDistToGoal(UnderwaterNavState &st) {
    long minDist = LONG_MAX;
    long minIdx = 0;
    long tmpDist;

    std::map<long, long>::iterator itM;
    for (itM = shortestDistToGoal_.begin(); itM != shortestDistToGoal_.end();
            itM++) {
        if ((tmpDist = dist(st, milestones_[itM->first])) < minDist) {
            minDist = tmpDist;
            minIdx = itM->first;
        }
    }
//cerr << "UseForHeuristic: " << minIdx << " " << shortestDistToGoal[minIdx] << endl;
    return shortestDistToGoal_[minIdx];
}

void StRoadmap::updateRoadmap(std::map<long, std::map<long, short> > &env_,
        std::vector<UnderwaterNavState> &goals, long nGoalsSamp) {
    env_ = env_;

    milestones_.clear();
    weight_.clear();
    std::map<long, std::vector<std::pair<long, long> > >::iterator itMap;
    for (itMap = outEdges_.begin(); itMap != outEdges_.end(); itMap++) {
        itMap->second.clear();
    }
    outEdges_.clear();
    for (itMap = inEdges_.begin(); itMap != inEdges_.end(); itMap++) {
        itMap->second.clear();
    }
    inEdges_.clear();

    setWeight();
    for (long i = 0; i < maxVerts_; i++) {
        insertGoalMilestones(goals, nGoalsSamp);
        UnderwaterNavState s(2);
        sampleAMilestone(s);
        insertMilestone(s);
    }
    getDistToGoal();
}

bool StRoadmap::hasMilestone(long x, long y) {
    UnderwaterNavState st(2);
    st[0] = x;
    st[1] = y;
    std::vector<UnderwaterNavState>::iterator itV;
    for (itV = milestones_.begin(); itV != milestones_.end(); itV++) {
        if (dist(st, *itV) == 0) {
            return true;
        }
    }
    return false;
}

void StRoadmap::draw(std::ostream &os) {
    std::map<long, std::map<long, short> >::iterator itCellType;
    os << endl;
    for (long y = 0; y < nRows_; y++) {
        for (long x = 0; x < nCols_; x++) {
            itCellType = env_.find(x);
            if (itCellType != env_.end()) {
                if (itCellType->second.find(y) != itCellType->second.end()) {
                    // 0: usual, 1: goals, 2: rocks, 3: observation, 4: spc. reward, 5: obstacle.
                    switch (env_[x][y]) {
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
                        os << env_[x][y];
                        break;
                    }
                    }
                } else if (hasMilestone(x, y)) {
                    os << "V";
                } else {
                    os << " ";
                }
            } else if (hasMilestone(x, y)) {
                os << "V";
            } else {
                os << " ";
            }
        }
        os << " " << endl;
    }
}
} /* namespace uwnav */
