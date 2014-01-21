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
class StateIndex;

class StatePool {
  public:
    friend class TextSerializer;

    StatePool(unsigned long nSDim);
    ~StatePool();
    StatePool(StatePool const &) = delete;
    StatePool(StatePool &&) = delete;
    StatePool &operator=(StatePool const &) = delete;
    StatePool &operator=(StatePool &&) = delete;

    void reset();
    StateInfo *add(std::unique_ptr<State> state);
    StateInfo *getStateById(long stId);

    StateIndex *getStateIndex();
    void addToStateIndex(StateInfo *stateInfo);

  private:
    unsigned long nSDim_;
    StateInfoOwningSet allStates_;
    std::vector<StateInfo *> allStatesIdx_;
    std::unique_ptr<StateIndex> stateIndex_;
};
} /* namespace solver */

#endif /* SOLVER_STATEPOOL_HPP_ */
