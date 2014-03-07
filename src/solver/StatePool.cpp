#include "StatePool.hpp"

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
    changedStates_.clear();
}

StateInfo *StatePool::add(std::unique_ptr<StateInfo> newInfo) {
    std::pair<StateInfoMap::iterator, bool> ret = (
            stateInfoMap_.emplace(newInfo->getState(), newInfo.get()));
    StateInfo *stateInfo = ret.first->second;
    if (ret.second) {
        long newId = long(statesByIndex_.size());
        // New state - add to the index.
        long oldId = stateInfo->getId();
        if (oldId != -1 && oldId != newId) {
            std::ostringstream message;
            message << "ERROR: ID mismatch - file says " << oldId;
            message << " but and ID of " << newId << " was assigned.";
            debug::show_message(message.str());
        }
        stateInfo->setId(newId);
        statesByIndex_.push_back(std::move(newInfo));
        addToStateIndex(stateInfo);
    } else {
        debug::show_message("ERROR: StateInfo already added!!");
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
