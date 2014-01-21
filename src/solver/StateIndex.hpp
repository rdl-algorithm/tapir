#ifndef STATESPATIALINDEX_HPP_
#define STATESPATIALINDEX_HPP_

#include <memory>

class StateInfo;
class IndexQuery;

namespace solver {
class StateIndex {
  public:
    StateIndex() = default;
    virtual ~StateIndex() = default;
    StateIndex(StateIndex const &) = delete;
    StateIndex(StateIndex &&) = delete;
    virtual StateIndex &operator=(StateIndex const &) = delete;
    virtual StateIndex &operator=(StateIndex &&) = delete;

    virtual void reset() = 0;

    virtual void addStateInfo(StateInfo *stateInfo) = 0;
    virtual void removeStateInfo(StateInfo *stateInfo) = 0;

    virtual StateInfo *getNearestNeighbor(StateInfo *stateInfo) = 0;
};
} /* namespace solver */

#endif /* STATESPATIALINDEX_HPP_ */
