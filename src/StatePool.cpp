#include "StatePool.hpp"

#include <algorithm>
using std::max;
using std::set_intersection;
#include <iterator>
using std::insert_iterator;
#include <map>
using std::multimap;
#include <set>
using std::set;
#include <utility>
using std::pair;
#include <vector>
using std::vector;

StatePool::StatePool() :
            nStates(0),
            nSDim(-1),
            allStates(),
            allStatesIdx(),
            stStruct() {
}

StatePool::~StatePool() {
    reset();
}

void StatePool::reset() {
    nStates = 0;
    for (StateWrapper *wrappedState : allStates) {
        delete wrappedState;
    }
    allStates.clear();
    allStatesIdx.clear();
    for (long i = 0; i < nSDim; i++) {
        stStruct[i].clear();
    }
    stStruct.clear();
}

StateWrapper* StatePool::add(State &sVals) {
    if (nSDim == -1) {
        nSDim = sVals.vals.size();
        stStruct.resize(nSDim);
    }
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

StateWrapper* StatePool::getStateById(long id) {
    return allStatesIdx[id];
}

void StatePool::identifyAffectedStates(State &lowLeft, State &upRight,
        ChangeType chType, set<StateWrapper*> &allAffectedSt) {
    multimap<double, StateWrapper*>::iterator start, end, it;
    /*
     cerr << "InStPool size of StStruct: " << stStruct.size() << " " << stStruct[0].size() << " low " << lowLeft[0] << " " << upRight[0] << endl;
     for (start = stStruct[0].begin(); start != stStruct[0].end(); start++) {
     cerr << "stStruct " << start->first << " " << start->second->s[0] << " " << start->second->s[1] << endl;
     }
     */
    start = stStruct[0].lower_bound(lowLeft.vals[0]);
    end = stStruct[0].lower_bound(upRight.vals[0]);
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
        start = stStruct[i].lower_bound(lowLeft.vals[i]);
        end = stStruct[i].lower_bound(upRight.vals[i]);
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
