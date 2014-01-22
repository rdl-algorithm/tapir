#ifndef SOLVER_STATESETS_HPP_
#define SOLVER_STATESETS_HPP_

#include <unordered_set>

#include "State.hpp"
#include "StateInfo.hpp"

namespace solver {

struct StateInfoHashUniquePtr {
    std::size_t operator()(std::unique_ptr<StateInfo> const &stateInfo) const {
        return stateInfo->getState()->hash();
    }
};
struct SameStateInfoUniquePtr {
    bool operator()(std::unique_ptr<StateInfo> const &s1,
            std::unique_ptr<StateInfo> const &s2) const {
        return *(s1->getState()) == *(s2->getState());
    }
};
typedef std::unordered_set<std::unique_ptr<StateInfo>, StateInfoHashUniquePtr,
        SameStateInfoUniquePtr> StateInfoOwningSet;



struct StateInfoHash {
    std::size_t operator()(StateInfo * const &stateInfo) const {
        return stateInfo->getState()->hash();
    }
};
struct SameStateInfo {
    bool operator()(StateInfo * const &s1, StateInfo * const &s2) const {
        return *(s1->getState()) == *(s2->getState());
    }
};
typedef std::unordered_set<StateInfo *, StateInfoHash, SameStateInfo> StateInfoSet;

} /* namespace solver */

#endif /* SOLVER_STATESETS_HPP_ */
