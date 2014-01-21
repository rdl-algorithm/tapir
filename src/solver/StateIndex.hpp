#ifndef STATESPATIALINDEX_HPP_
#define STATESPATIALINDEX_HPP_

#include <memory>

namespace solver {
class StateInfo;

class StateIndex {
  public:
    StateIndex() = default;
    virtual ~StateIndex() = default;
    StateIndex(StateIndex const &) = delete;
    StateIndex(StateIndex &&) = delete;
    virtual StateIndex &operator=(StateIndex const &) = delete;
    virtual StateIndex &operator=(StateIndex &&) = delete;

    /** Resets the state index, making it empty. */
    virtual void reset() = 0;
    /** Adds the given state info to the index. */
    virtual void addStateInfo(StateInfo *stateInfo) = 0;
    /** Removes the given state info from the index. */
    virtual void removeStateInfo(StateInfo *stateInfo) = 0;
};
} /* namespace solver */

#endif /* STATESPATIALINDEX_HPP_ */
