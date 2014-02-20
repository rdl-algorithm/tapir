#ifndef SOLVER_STATEINDEX_HPP_
#define SOLVER_STATEINDEX_HPP_

#include <memory>

#include "global.hpp"

namespace solver {
class StateInfo;

class StateIndex {
  public:
    StateIndex() = default;
    virtual ~StateIndex() = default;
    _NO_COPY_OR_MOVE(StateIndex);

    /** Resets the state index, making it empty. */
    virtual void reset() = 0;
    /** Adds the given state info to the index. */
    virtual void addStateInfo(StateInfo *stateInfo) = 0;
    /** Removes the given state info from the index. */
    virtual void removeStateInfo(StateInfo *stateInfo) = 0;
};
} /* namespace solver */

#endif /* SOLVER_STATEINDEX_HPP_ */
