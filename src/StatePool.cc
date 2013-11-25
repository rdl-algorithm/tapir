#include <sstream>
#include <string>
#include <algorithm>

#include <cstdlib>

#include "StatePool.h"

using namespace std;

StatePool::StatePool() :
            nStates(0),
            nSDim(-1),
            allStates(),
            allStatesIdx(),
            stStruct() {
}

StatePool::~StatePool() {
    set<StateWrapper*>::iterator it;
    for (it = allStates.begin(); it != allStates.end(); it++) {
        delete (*it);
    }
    allStates.clear();
    allStatesIdx.clear();
    for (long i = 0; i < nSDim; i++) {
        stStruct[i].clear();
    }
    stStruct.clear();
}

void StatePool::readStates(ifstream &inFile, Model *model) {
    nSDim = model->getNStVars();
    stStruct.resize(nSDim);

    string tmpStr, usrStr;
    getline(inFile, tmpStr);
    while (tmpStr.find("STATESPOOL-BEGIN") == string::npos) {
        getline(inFile, tmpStr);
    }

    getline(inFile, tmpStr);
    stringstream sstr(tmpStr);
    sstr >> usrStr >> nStates;
    getline(inFile, tmpStr);

    for (long i = 0; i < nStates; i++) {
        allStatesIdx.push_back(nullptr);
    }
    pair<set<StateWrapper*, CompStVals>::iterator, bool> ret;
    while (tmpStr.find("STATESPOOL-END") == string::npos) {
        StateWrapper *newSt = new StateWrapper(tmpStr, nSDim);
        ret = allStates.insert(newSt);
        allStatesIdx[newSt->id] = *(ret.first);
        for (long i = 0; i < nSDim; i++) {
            stStruct[i].insert(
                    pair<double, StateWrapper*>(newSt->s[i], *(ret.first)));
        }
        getline(inFile, tmpStr);
    }
}

StateWrapper* StatePool::add(StateVals &sVals) {
    StateWrapper* newSt = new StateWrapper(sVals);
    pair<set<StateWrapper*, CompStVals>::iterator, bool> ret = allStates.insert(
            newSt);
    if (ret.second) {
        newSt->setId();
        allStatesIdx.push_back(newSt);
        nStates++;
        return newSt;
    } else {
        delete newSt;
        return *(ret.first);
    }
}

StateWrapper* StatePool::getStPtr(long stId) {
    return allStatesIdx[stId];
}

void StatePool::identifyAffectedStates(StateVals &lowLeft, StateVals &upRight,
        Change chType, set<StateWrapper*> &allAffectedSt) {
    multimap<double, StateWrapper*>::iterator start, end, it;
    /*
     cerr << "InStPool size of StStruct: " << stStruct.size() << " " << stStruct[0].size() << " low " << lowLeft[0] << " " << upRight[0] << endl;
     for (start = stStruct[0].begin(); start != stStruct[0].end(); start++) {
     cerr << "stStruct " << start->first << " " << start->second->s[0] << " " << start->second->s[1] << endl;
     }
     */
    start = stStruct[0].lower_bound(lowLeft[0]);
    end = stStruct[0].lower_bound(upRight[0]);
    /*
     if (start == stStruct[0].end()) { cerr << "No Start\n"; }
     if (end == stStruct[0].end()) { cerr << "No End\n"; }
     cerr << "StartEndStVals " << start->second->s[0] << " " << start->second->s[1] << " to " <<
     end->second->s[0] << " " << end->second->s[1] << endl;
     */
    set<StateWrapper*> affectedSt;
    for (it = start; it != end; it++) {
        /*
         if (it->first == 15) {
         cerr << "St " << it->second->s[0] << " " << it->second->s[1] << endl;
         }
         */
        affectedSt.insert(it->second);
    }
    //cerr << "#affectedSt: " << affectedSt.size() << endl;
    //cerr << "ok1\n";
    for (long i = 1; i < nSDim; i++) {
        start = stStruct[i].lower_bound(lowLeft[i]);
        end = stStruct[i].lower_bound(upRight[i]);
        set<StateWrapper*> posAffectedSt, tmpSet;
        for (it = start; it != end; it++) {
            posAffectedSt.insert(it->second);
        }
        //cerr << "#affectedSt for dim-" << i << " : " << posAffectedSt.size() << endl;
        set_intersection(affectedSt.begin(), affectedSt.end(),
                posAffectedSt.begin(), posAffectedSt.end(),
                insert_iterator<set<StateWrapper*> >(tmpSet, tmpSet.begin()));
        affectedSt = tmpSet;
        //cerr << "#IntersectAffectedSt for dim-" << i << " : " << affectedSt.size() << endl;
    }
    //cerr << "ok2\n";
    set<StateWrapper*>::iterator itSt;
    for (itSt = affectedSt.begin(); itSt != affectedSt.end(); itSt++) {
        (*itSt)->chType = max((*itSt)->chType, chType);
    }
    for (itSt = affectedSt.begin(); itSt != affectedSt.end(); itSt++) {
        allAffectedSt.insert(*itSt);
    }
    //cerr << "ok3\n";
}

void StatePool::write(ostream &os) {
    os << "nStates: " << nStates << endl;
    set<StateWrapper*, CompStVals>::iterator it;
    for (it = allStates.begin(); it != allStates.end(); it++) {
        (*it)->writeln(os);
    }
}
