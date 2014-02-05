#include "StatePool.hpp"

#include <iostream>                     // for operator<<, basic_ostream, cerr, endl, ostream
#include <map>                          // for multimap, __alloc_traits<>::value_type
#include <set>                          // for set
#include <unordered_map>                // for unordered_map
#include <unordered_set>                // for unordered_set
#include <utility>                      // for move, pair

#include "defs.hpp"                     // for make_unique

#include "ChangeFlags.hpp"               // for ChangeFlags
#include "RTree.hpp"
#include "StateInfo.hpp"                // for StateInfo, StateInfo::currId

namespace solver {
class State;

StatePool::StatePool(unsigned long nSDim) :
    nSDim_(nSDim),
    stateInfoMap_(),
    statesByIndex_(),
    stateIndex_(std::make_unique<RTree>(nSDim_, this)),
    changedStates_() {
}

StatePool::~StatePool() {
}

StateInfo *StatePool::getInfo(State *state) {
    StateInfoMap::iterator it = stateInfoMap_.find(state);
    if (it == stateInfoMap_.end()) {
        return nullptr;
    }
    return it->second;
}

StateInfo *StatePool::getInfoById(long id) {
    return statesByIndex_[id].get();
}

void StatePool::reset() {
    stateInfoMap_.clear();
    statesByIndex_.clear();
    stateIndex_->reset();
    StateInfo::currId = 0;
    changedStates_.clear();
}

StateInfo *StatePool::add(std::unique_ptr<StateInfo> newInfo) {
    std::pair<StateInfoMap::iterator, bool> ret = (
            stateInfoMap_.emplace(newInfo->getState(), newInfo.get()));
    StateInfo *stateInfo = ret.first->second;
    if (ret.second) {
        // New state - add to the index.
        stateInfo->setId();
        if (stateInfo->id_ != (long)statesByIndex_.size()) {
            std::cerr << "Error: wrong size in StatePool?" << std::endl;
        }
        statesByIndex_.push_back(std::move(newInfo));
        addToStateIndex(stateInfo);
    } else {
        std::cerr << "StateInfo already added!!" << std::endl;
    }
    return stateInfo;
}

StateInfo *StatePool::createOrGetInfo(std::unique_ptr<State> state) {
    StateInfo *info = getInfo(state.get());
    if (info != nullptr) {
        return info;
    }
    return add(std::make_unique<StateInfo>(std::move(state)));
}

StateIndex *StatePool::getStateIndex() {
    return stateIndex_.get();
}

void StatePool::addToStateIndex(StateInfo *stateInfo) {
    stateIndex_->addStateInfo(stateInfo);
}

void StatePool::resetChangeFlags(StateInfo *stateInfo) {
    stateInfo->resetChangeFlags();
    changedStates_.erase(stateInfo);
}

void StatePool::setChangeFlags(StateInfo *stateInfo, ChangeFlags flags) {
    if (flags != ChangeFlags::UNCHANGED) {
        stateInfo->setChangeFlags(flags);
        changedStates_.insert(stateInfo);
    }
}

void StatePool::resetAffectedStates() {
    for (StateInfo *stateInfo : changedStates_) {
        stateInfo->resetChangeFlags();
    }
    changedStates_.clear();
}

std::unordered_set<StateInfo *> StatePool::getAffectedStates() {
    return changedStates_;
}
} /* namespace solver */
