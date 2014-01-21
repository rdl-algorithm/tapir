#include "StatePool.hpp"

#include <iostream>                     // for operator<<, basic_ostream, cerr, endl, ostream
#include <map>                          // for multimap, __alloc_traits<>::value_type
#include <set>                          // for set
#include <utility>                      // for move, pair

#include "defs.hpp"                     // for make_unique

#include "ChangeType.hpp"               // for ChangeType
#include "RTree.hpp"
#include "StateInfo.hpp"                // for StateInfo, StateInfo::currId

namespace solver {
class State;

StatePool::StatePool(unsigned long nSDim) :
    nSDim_(nSDim),
    allStates_(),
    allStatesIdx_(),
    stateIndex_(std::make_unique<RTree>(nSDim_, this)) {
}

StatePool::~StatePool() {
}

void StatePool::reset() {
    allStates_.clear();
    allStatesIdx_.clear();
    stateIndex_->reset();
    StateInfo::currId = 0;
}

StateInfo *StatePool::add(std::unique_ptr<State> state) {
    std::unique_ptr<StateInfo> newInfo = std::make_unique<StateInfo>(
                std::move(state));
    StateInfo *stateInfo = newInfo.get();
    std::pair<StateInfoOwningSet::iterator,bool> ret = allStates_.insert(
            std::move(newInfo));
    if (ret.second) {
        stateInfo->setId();
        if (stateInfo->id_ != (long)allStatesIdx_.size()) {
            std::cerr << "Error: wrong size in StatePool" << std::endl;
        }
        allStatesIdx_.push_back(stateInfo);
        addToStateIndex(stateInfo);
    } else {
        stateInfo = ret.first->get();
    }
    return stateInfo;
}

StateInfo *StatePool::getStateById(long id) {
    return allStatesIdx_[id];
}


StateIndex *StatePool::getStateIndex() {
    return stateIndex_.get();
}

void StatePool::addToStateIndex(StateInfo *stateInfo) {
    stateIndex_->addStateInfo(stateInfo);
}
} /* namespace solver */
