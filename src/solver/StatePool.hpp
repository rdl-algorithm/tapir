#ifndef SOLVER_STATEPOOL_HPP_
#define SOLVER_STATEPOOL_HPP_

#include <cstddef>                      // for size_t

#include <map>                          // for multimap
#include <memory>                       // for unique_ptr
#include <set>                          // for set
#include <unordered_set>                // for unordered_set
#include <vector>                       // for vector

#include "ChangeType.hpp"               // for ChangeType
#include "State.hpp"                    // for State, operator==
#include "StateInfo.hpp"                // for StateInfo
#include "StateSets.hpp"

namespace solver {
class StatePool {
  public:
    friend class TextSerializer;

    StatePool();
    ~StatePool();
    StatePool(StatePool const &) = delete;
    StatePool(StatePool &&) = delete;
    StatePool &operator=(StatePool const &) = delete;
    StatePool &operator=(StatePool &&) = delete;

    void reset();
    StateInfo *add(std::unique_ptr<State> state);
    StateInfo *getStateById(long stId);
    void identifyAffectedStates(State &lowLeft, State &upRight,
            ChangeType chType, std::set<StateInfo *> &affectedSt);

  private:
    long nSDim_;
    StateInfoOwningSet allStates_;
    std::vector<StateInfo *> allStatesIdx_;
    std::vector<std::multimap<double, StateInfo * >> stStruct_;
};
} /* namespace solver */

#endif /* SOLVER_STATEPOOL_HPP_ */
