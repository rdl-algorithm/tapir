#include "StatePool.hpp"

#include <iostream>                     // for operator<<, basic_ostream, cerr, endl, ostream
#include <map>                          // for multimap, __alloc_traits<>::value_type
#include <set>                          // for set
#include <utility>                      // for move, pair

#include "defs.hpp"                     // for make_unique

#include "ChangeType.hpp"               // for ChangeType
#include "StateInfo.hpp"                // for StateInfo, StateInfo::currId

namespace solver {
class State;

StatePool::StatePool() :
    nSDim_(-1),
    allStates_(),
    allStatesIdx_(),
    stStruct_() {
}

StatePool::~StatePool() {
}

void StatePool::reset() {
    allStates_.clear();
    allStatesIdx_.clear();
    for (long i = 0; i < nSDim_; i++) {
        stStruct_[i].clear();
    }
    stStruct_.clear();
    StateInfo::currId = 0;
}

StateInfo *StatePool::add(std::unique_ptr<State> state) {
//    if (nSDim == -1) {
//        nSDim = sVals.vals.size();
//        stStruct.resize(nSDim);
//    }
    std::unique_ptr<StateInfo> newInfo = std::make_unique<StateInfo>(
                std::move(state));
    StateInfo *stateInfo = newInfo.get();
    std::pair<StateInfoSet::iterator,
            bool> ret = allStates_.insert(std::move(newInfo));
    if (ret.second) {
        stateInfo->setId();
        if (stateInfo->id_ != (long)allStatesIdx_.size()) {
            std::cerr << "Error: wrong size in StatePool" << std::endl;
        }
        allStatesIdx_.push_back(stateInfo);
    } else {
        stateInfo = ret.first->get();
    }
    return stateInfo;
}

StateInfo *StatePool::getStateById(long id) {
    return allStatesIdx_[id];
}

void StatePool::identifyAffectedStates(State & /*lowLeft*/, State & /*upRight*/,
        ChangeType /*chType*/, std::set<StateInfo *> & /*allAffectedSt*/) {
//    std::multimap<double, StateInfo *>::iterator start, end, it;
//    /*
//     cerr << "InStPool size of StStruct: " << stStruct.size() << " " << stStruct[0].size() << " low " << lowLeft[0] << " " << upRight[0] << endl;
//     for (start = stStruct[0].begin(); start != stStruct[0].end(); start++) {
//     cerr << "stStruct " << start->first << " " << start->second->s[0] << " " << start->second->s[1] << endl;
//     }
//     */
//    start = stStruct[0].lower_bound(lowLeft.vals[0]);
//    end = stStruct[0].lower_bound(upRight.vals[0]);
//    /*
//     if (start == stStruct[0].end()) { cerr << "No Start\n"; }
//     if (end == stStruct[0].end()) { cerr << "No End\n"; }
//     cerr << "StartEndStVals " << start->second->s[0] << " " << start->second->s[1] << " to " <<
//     end->second->s[0] << " " << end->second->s[1] << endl;
//     */
//    std::set<StateInfo *> affectedSt;
//    for (it = start; it != end; it++) {
//        /*
//         if (it->first == 15) {
//         cerr << "St " << it->second->s[0] << " " << it->second->s[1] << endl;
//         }
//         */
//        affectedSt.insert(it->second);
//    }
//    //cerr << "#affectedSt: " << affectedSt.size() << endl;
//    //cerr << "ok1\n";
//    for (long i = 1; i < nSDim; i++) {
//        start = stStruct[i].lower_bound(lowLeft.vals[i]);
//        end = stStruct[i].lower_bound(upRight.vals[i]);
//        std::set<StateInfo *> posAffectedSt, tmpSet;
//        for (it = start; it != end; it++) {
//            posAffectedSt.insert(it->second);
//        }
//        //cerr << "#affectedSt for dim-" << i << " : " << posAffectedSt.size() << endl;
//        std::set_intersection(affectedSt.begin(), affectedSt.end(),
//                posAffectedSt.begin(), posAffectedSt.end(),
//                std::insert_iterator<std::set<StateInfo *> >(tmpSet,
//                        tmpSet.begin()));
//        affectedSt = tmpSet;
//        //cerr << "#IntersectAffectedSt for dim-" << i << " : " << affectedSt.size() << endl;
//    }
//    //cerr << "ok2\n";
//    std::set<StateInfo *>::iterator itSt;
//    for (itSt = affectedSt.begin(); itSt != affectedSt.end(); itSt++) {
//        (*itSt)->chType = std::max((*itSt)->chType, chType);
//    }
//    for (itSt = affectedSt.begin(); itSt != affectedSt.end(); itSt++) {
//        allAffectedSt.insert(*itSt);
//    }
//    //cerr << "ok3\n";
}
} /* namespace solver */
