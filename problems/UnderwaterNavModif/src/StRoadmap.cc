#include <cmath>
#include <climits>

#include "StRoadmap.h"

using namespace std;

StRoadmap::StRoadmap(vector<StateVals> &goals, long maxVerts, long nGoalsSamp,
        long nTryCon, long maxDistCon, map<long, map<long, short> > &env,
        long nX, long nY) :
        maxVerts(maxVerts), maxTryCon(nTryCon), maxDistCon(maxDistCon), env(
                env), nX(nX), nY(nY) {
    setWeight();
    nVerts = 0;
    insertGoalMilestones(goals, nGoalsSamp);
    long i = lastGoalIdx;

    //insertMyMilestones();

    while (i < maxVerts) {
        StateVals s(2);
        sampleAMilestone(s);
        if (insertMilestone(s)) {
            i++;
        }
    }
    /*
     vector<StateVals>::iterator itV;
     cerr << "Vertices: (lastGoal "  << lastGoalIdx << ")\n";
     i=0;
     for (itV = V.begin(); itV != V.end(); itV++, i++) {
     cerr << "v-" << i << " " << (*itV)[0] << " " << (*itV)[1] << endl;
     }
     cerr << "Edges:\n";
     map< long, vector< pair<long, long> > >::iterator it1;
     vector< pair<long, long> >::iterator it2;
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
     map<long, long>::iterator itM;
     for (itM = shortestDistToGoal.begin(); itM != shortestDistToGoal.end(); itM++) {
     cerr << "distToGoal From " << itM->first << " cost " << itM->second << endl;
     }
     */
//  draw(cerr);
}

StRoadmap::~StRoadmap() {
}

void StRoadmap::setWeight() {
    map<long, map<long, short> >::iterator itX;
    map<long, short>::iterator itY;
    totW = 0;
    long prevX = 0;
    long prevY, pivIdx;
    for (itX = env.begin(); itX != env.end(); itX++) {
        for (long x = prevX; x < itX->first; x++) {
//cerr << "Fr " << prevX << " to " << itX->first << " curr " << x
//      << " prevY " << prevY << " to " << itY->first << endl;
            pivIdx = x * nY;
            for (long y = 0; y < nY; y++) {
                totW++;
                weight[totW] = pivIdx + y;
//cerr << "LeftOverY weight[" << totW << "] : " << x << " " << y << " " << weight[totW] << endl;
            }
        }
//cerr << "At " << itX->first << endl;
        prevY = 0;
        pivIdx = itX->first * nY;
        for (itY = itX->second.begin(); itY != itX->second.end(); itY++) {
            for (long y = prevY; y < itY->first; y++) {
                totW++;
                weight[totW] = pivIdx + y;
//cerr << "weight[" << totW << "] : " << itX->first << " " << y << " " << weight[totW] << endl;
            }
            if (itY->second == 3) { // Observation
//cerr << "OBS ( " << itX->first << " " << itY->first << " ) ";
                totW++;
                weight[totW] = pivIdx + itY->first;
//cerr << "weight[" << totW << "] : " << itX->first << " " << itY->first << " " << weight[totW] << " ";
                totW++;
                weight[totW] = pivIdx + itY->first;
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
            prevY = itY->first + 1;
        }
        for (long y = prevY; y < nY; y++) {
            totW++;
            weight[totW] = pivIdx + y;
//cerr << "LeftOverY weight[" << totW << "] : " << itX->first << " " << y << " " << weight[totW] << endl;
        }

        prevX = itX->first + 1;
    }
    for (long x = 0; x < env.begin()->first; x++) {
        pivIdx = x * nY;
        for (long y = 0; y < nY; y++) {
            totW++;
            weight[totW] = pivIdx + y;
//cerr << "LeftOverX weight[" << totW << "] : " << x << " " << y << " " << weight[totW] << endl;
        }
    }
    for (long x = prevX; x < nX; x++) {
        pivIdx = x * nY;
        for (long y = 0; y < nY; y++) {
            totW++;
            weight[totW] = pivIdx + y;
//cerr << "LeftOverX weight[" << totW << "] : " << x << " " << y << " " << weight[totW] << endl;
        }
    }
}

void StRoadmap::insertGoalMilestones(vector<StateVals> &goals,
        long nGoalsSamp) {
    long nGoals = goals.size();
    //for (long i = 0; i < nGoalsSamp; i++) {
    vector<StateVals>::iterator itV;
    bool inserted;
    while (nVerts < nGoalsSamp) {
        long rawIdx = GlobalResources::randIntBetween(0, nGoals - 1);
        StateVals st(2);
        st[0] = goals[rawIdx][0];
        st[1] = goals[rawIdx][1];
        inserted = true;
        for (itV = V.begin(); itV != V.end(); itV++) {
            if (dist(st, *itV) == 0) {
                inserted = false;
                break;
            }
        }
        if (inserted) {
            V.push_back(st);
            nVerts++;
        }
    }
    lastGoalIdx = nVerts - 1;
}

void StRoadmap::insertMyMilestones() {
    vector<StateVals> s;
    StateVals st(2);

    st[0] = 14;
    st[1] = 20;
    s.push_back(st);
    st[0] = 14;
    st[1] = 40;
    s.push_back(st);
    st[0] = 24;
    st[1] = 20;
    s.push_back(st);
    st[0] = 24;
    st[1] = 40;
    s.push_back(st);
    st[0] = 33;
    st[1] = 18;
    s.push_back(st);
    st[0] = 41;
    st[1] = 18;
    s.push_back(st);

    vector<StateVals>::iterator it;
    for (it = s.begin(); it != s.end(); it++) {
        insertMilestone(*it);
    }
}

void StRoadmap::sampleAMilestone(StateVals &st) {
    long rawIdx = GlobalResources::randIntBetween(1, totW);
    st[0] = (long) floor(weight[rawIdx] / nY);
    st[1] = weight[rawIdx] % nY;
    /*
     if (st[0] >= 35 && st[0] < 39 && st[1] >= 20 && st[1] < 52) {
     cerr << "SAMPLE ROCK raw " << rawIdx << " w " << weight[rawIdx] << " env "
     << env[st[0]][st[1]] << " ( " << st[0] << " " << st[1] << " ) "
     << weight[rawIdx-1] << " " << weight[rawIdx+1] << endl;
     }
     */
}

bool StRoadmap::insertMilestone(StateVals &st) {
    long c, i;
    vector<StateVals>::iterator itV;
    for (itV = V.begin(), i = 0; itV != V.end(); itV++, i++) {
        if ((c = dist(st, *itV)) == 0) {
            return false;
        }
    }

    long nTry = 0;
    for (itV = V.begin(), i = 0; itV != V.end(); itV++, i++) {
        if (dist(st, *itV) < maxDistCon && nTry < maxTryCon
                && (c = lineSegOk(st, *itV)) > -1) {
            outEdges[nVerts].push_back(make_pair(i, c));
            inEdges[i].push_back(make_pair(nVerts, c));
        }
        nTry++;
    }
    V.push_back(st);
    nVerts++;
    return true;
}

double StRoadmap::dist(StateVals &s1, StateVals &s2) {
    return (fabs(s2[0] - s1[0]) + fabs(s2[1] - s1[1]));
}

long StRoadmap::lineSegOk(StateVals &st1, StateVals &st2) {
    map<long, map<long, short> >::iterator itX;
    map<long, short>::iterator itY;

    // Move until Y = end Y position, and then move in X direction.
    if ((itX = env.find(st1[0])) == env.end()) {
        for (long x = st1[0]; x < st2[0]; x++) {
            itX = env.find(x);
            if (itX != env.end()) {
                if ((itY = itX->second.find(st2[1])) != itX->second.end()) {
                    if (itY->second == 2 || itY->second == 5) {
                        return -1;
                    }
                }
            }
        }
        return (fabs(st2[0] - st1[0]) + fabs(st2[1] - st1[1]));
    } else {
        for (long y = st1[1]; y < st2[2]; y++) {
            if ((itY = itX->second.find(y)) != itX->second.end()) {
                if (itY->second == 2 || itY->second == 5) {
                    return -1;
                }
            }
        }
        for (long x = st1[0]; x < st2[0]; x++) {
            itX = env.find(x);
            if (itX != env.end()) {
                if ((itY = itX->second.find(st2[1])) != itX->second.end()) {
                    if (itY->second == 2 || itY->second == 5) {
                        return -1;
                    }
                }
            }
        }
        return (fabs(st2[0] - st1[0]) + fabs(st2[1] - st1[1]));
    }

    // Move until Y = end Y position, and then move in X direction.
    if ((itX = env.find(st2[0])) == env.end()) {
        for (long x = st1[0]; x < st2[0]; x++) {
            itX = env.find(x);
            if (itX != env.end()) {
                if ((itY = itX->second.find(st1[1])) != itX->second.end()) {
                    if (itY->second == 2 || itY->second == 5) {
                        return -1;
                    }
                }
            }
        }
        return (fabs(st2[0] - st1[0]) + fabs(st2[1] - st1[1]));
    } else {
        for (long y = st1[1]; y < st2[2]; y++) {
            if ((itY = itX->second.find(y)) != itX->second.end()) {
                if (itY->second == 2 || itY->second == 5) {
                    return -1;
                }
            }
        }
        for (long x = st1[0]; x < st2[0]; x++) {
            itX = env.find(x);
            if (itX != env.end()) {
                if ((itY = itX->second.find(st1[1])) != itX->second.end()) {
                    if (itY->second == 2 || itY->second == 5) {
                        return -1;
                    }
                }
            }
        }
        return (fabs(st2[0] - st1[0]) + fabs(st2[1] - st1[1]));
    }
}

// Dijkstra shortest path.
void StRoadmap::getDistToGoal() {
    vector<pair<long, long> >::iterator itNxt;
    long tmpDist;

    // Find shortest path to each goal
    vector<vector<long> > allDist;
    for (long i = 0; i <= lastGoalIdx; i++) {
        vector<long> dist(nVerts, LONG_MAX);
        vector<long> prev(nVerts);
        vector<bool> visited(nVerts, false);

        for (long j = 0; j <= lastGoalIdx; j++) {
            dist[j] = 0;
            visited[j] = true;
        }

        // Iteration
        multimap<long, long> q;
        map<long, multimap<long, long>::iterator> ptrToIdx;
        for (int j = lastGoalIdx + 1; j < nVerts; j++) {
            ptrToIdx[j] = q.insert(pair<long, long>(LONG_MAX, j));
        }
        for (int j = 0; j <= lastGoalIdx; j++) {
            ptrToIdx[j] = q.end();
        }
        for (itNxt = inEdges[i].begin(); itNxt != inEdges[i].end(); itNxt++) {
//cerr << "About to erase " << itNxt->first << " of v " << i << endl;
            q.erase(ptrToIdx[itNxt->first]);
            ptrToIdx[itNxt->first] = q.insert(
                    pair<long, long>(itNxt->second, itNxt->first));
            dist[itNxt->first] = itNxt->second;
            //nxt[itNxt->first] = i;
        }
        long currIdx;
        multimap<long, long>::iterator itQ;
        while (!q.empty()) {
            itQ = q.begin();
            currIdx = itQ->second;
            for (itNxt = inEdges[currIdx].begin();
                    itNxt != inEdges[currIdx].end(); itNxt++) {
                if (!visited[itNxt->first]
                        && (tmpDist = itQ->first + itNxt->second)
                                < ptrToIdx[itNxt->first]->first) {
                    q.erase(ptrToIdx[itNxt->first]);
                    ptrToIdx[itNxt->first] = q.insert(
                            pair<long, long>(tmpDist, itNxt->first));
                    dist[itNxt->first] = tmpDist;
                    //nxt[itNxt->first] = currIdx;
                }
            }
            visited[currIdx] = true;
            q.erase(itQ);
        }
        /*
         vector<long>::iterator itD;
         cerr << "Dist ";
         for (itD = dist.begin(); itD != dist.end(); itD++) {
         cerr << *itD << " ";
         }
         cerr << endl;
         */
        allDist.push_back(dist);
    }

    vector<vector<long> >::iterator itVecVecL = allDist.begin();
    vector<long>::iterator itVecL;
    for (long i = 0; i < nVerts; i++) {
        shortestDistToGoal[i] = LONG_MAX;
    }
    for (itVecVecL->begin(); itVecVecL != allDist.end(); itVecVecL++) {
        long i = 0;
        for (itVecL = itVecVecL->begin(); itVecL != itVecVecL->end();
                itVecL++, i++) {
            if (*itVecL > 0 && *itVecL < shortestDistToGoal[i]) {
                shortestDistToGoal[i] = *itVecL;
            }
        }
    }

    for (long i = 0; i <= lastGoalIdx; i++) {
        shortestDistToGoal[i] = 0;
    }
    for (long i = lastGoalIdx + 1; i < nVerts; i++) {
        if (shortestDistToGoal[i] == LONG_MAX) { // For speed, we don't bother to erase the graph.
            shortestDistToGoal.erase(i);
        }
    }
}

double StRoadmap::getDistToGoal(StateVals &st) {
    long minDist = LONG_MAX;
    long minIdx = 0;
    long tmpDist;

    map<long, long>::iterator itM;
    for (itM = shortestDistToGoal.begin(); itM != shortestDistToGoal.end();
            itM++) {
        if ((tmpDist = dist(st, V[itM->first])) < minDist) {
            minDist = tmpDist;
            minIdx = itM->first;
        }
    }
//cerr << "UseForHeuristic: " << minIdx << " " << shortestDistToGoal[minIdx] << endl;
    return shortestDistToGoal[minIdx];
}

void StRoadmap::updateRoadmap(map<long, map<long, short> > &env_,
        vector<StateVals> &goals, long nGoalsSamp) {
    env = env_;

    V.clear();
    weight.clear();
    map<long, vector<pair<long, long> > >::iterator itMap;
    for (itMap = outEdges.begin(); itMap != outEdges.end(); itMap++) {
        itMap->second.clear();
    }
    outEdges.clear();
    for (itMap = inEdges.begin(); itMap != inEdges.end(); itMap++) {
        itMap->second.clear();
    }
    inEdges.clear();

    setWeight();
    for (long i = 0; i < maxVerts; i++) {
        insertGoalMilestones(goals, nGoalsSamp);
        StateVals s(2);
        sampleAMilestone(s);
        insertMilestone(s);
    }
    getDistToGoal();
}

bool StRoadmap::VContains(long x, long y) {
    StateVals st(2);
    st[0] = x;
    st[1] = y;
    vector<StateVals>::iterator itV;
    for (itV = V.begin(); itV != V.end(); itV++) {
        if (dist(st, *itV) == 0) {
            return true;
        }
    }
    return false;
}

void StRoadmap::draw(ostream &os) {
    map<long, map<long, short> >::iterator itCellType;
    os << endl;
    for (long y = 0; y < nY; y++) {
        for (long x = 0; x < nX; x++) {
            itCellType = env.find(x);
            if (itCellType != env.end()) {
                if (itCellType->second.find(y) != itCellType->second.end()) {
                    // 0: usual, 1: goals, 2: rocks, 3: observation, 4: spc. reward, 5: obstacle.
                    switch (env[x][y]) {
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
                        os << env[x][y];
                        break;
                    }
                    }
                } else if (VContains(x, y)) {
                    os << "V";
                } else {
                    os << " ";
                }
            } else if (VContains(x, y)) {
                os << "V";
            } else {
                os << " ";
            }
        }
        os << " " << endl;
    }
}
