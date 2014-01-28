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
    statesByIndex_(),
    stateIndex_(std::make_unique<RTree>(nSDim_, this)) {
}

StatePool::~StatePool() {
}

void StatePool::reset() {
    allStates_.clear();
    statesByIndex_.clear();
    stateIndex_->reset();
    StateInfo::currId = 0;
}

StateInfo *StatePool::add(std::unique_ptr<StateInfo> newInfo) {
    std::pair<StateInfoOwningSet::iterator, bool> ret = allStates_.insert(
            std::move(newInfo));
    StateInfo *stateInfo = ret.first->get();
    if (ret.second) {
        // New state - add its info.
        stateInfo->setId();
        if (stateInfo->id_ != (long)statesByIndex_.size()) {
            std::cerr << "Error: wrong size in StatePool" << std::endl;
        }
        statesByIndex_.push_back(stateInfo);
        addToStateIndex(stateInfo);
    }
    return stateInfo;
}

StateInfo *StatePool::add(std::unique_ptr<State> state) {
    return add(std::make_unique<StateInfo>(std::move(state)));

}

StateInfo *StatePool::getStateById(long id) {
    return statesByIndex_[id];
}

StateIndex *StatePool::getStateIndex() {
    return stateIndex_.get();
}

void StatePool::addToStateIndex(StateInfo *stateInfo) {
    stateIndex_->addStateInfo(stateInfo);
}
} /* namespace solver */
