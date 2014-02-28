#include "StatePool.hpp"

#include <iostream>                     // for operator<<, basic_ostream, cerr, endl, ostream
#include <map>                          // for multimap, __alloc_traits<>::value_type
#include <set>                          // for set
#include <unordered_map>                // for unordered_map
#include <unordered_set>                // for unordered_set
#include <utility>                      // for move, pair

#include "global.hpp"                     // for make_unique

#include "ChangeFlags.hpp"               // for ChangeFlags
#include "Model.hpp"
#include "geometry/State.hpp"
#include "StateInfo.hpp"                // for StateInfo, StateInfo::currId

namespace solver {

StatePool::StatePool(std::unique_ptr<StateIndex> stateIndex) :
    stateInfoMap_(),
    statesByIndex_(),
    stateIndex_(std::move(stateIndex)),
    changedStates_() {
}

StatePool::~StatePool() {
}

StateInfo *StatePool::getInfo(State const &state) const {
    StateInfoMap::const_iterator it = stateInfoMap_.find(&state);
    if (it == stateInfoMap_.end()) {
        return nullptr;
    }
    return it->second;
}

StateInfo *StatePool::getInfoById(long id) const {
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
            std::cerr << "ERROR: Wrong size in StatePool?" << std::endl;
        }
        statesByIndex_.push_back(std::move(newInfo));
        addToStateIndex(stateInfo);
    } else {
        std::cerr << "StateInfo already added!!" << std::endl;
    }
    return stateInfo;
}

StateInfo *StatePool::createOrGetInfo(State const &state) {
    StateInfo *info = getInfo(state);
    if (info != nullptr) {
        return info;
    }
    return add(std::make_unique<StateInfo>(state.copy()));
}

StateIndex *StatePool::getStateIndex() const {
    return stateIndex_.get();
}

void StatePool::addToStateIndex(StateInfo *stateInfo) const {
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

std::unordered_set<StateInfo *> StatePool::getAffectedStates() const {
    return changedStates_;
}
} /* namespace solver */
