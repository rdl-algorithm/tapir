#ifndef STATESPATIALINDEX_HPP_
#define STATESPATIALINDEX_HPP_

#include <memory>

class StateInfo;
class StateQuery;

namespace solver {
class StateSpatialIndex {
  public:
    StateSpatialIndex() = default;
    virtual ~StateSpatialIndex() = default;
    StateSpatialIndex(StateSpatialIndex const &) = delete;
    StateSpatialIndex(StateSpatialIndex &&) = delete;
    virtual StateSpatialIndex &operator=(StateSpatialIndex const &) = delete;
    virtual StateSpatialIndex &operator=(StateSpatialIndex &&) = delete;

    virtual void addStateInfo(StateInfo *stateInfo) = 0;
    virtual void removeStateInfo(StateInfo *stateInfo) = 0;
    virtual StateInfo *getNearestNeighbor(StateInfo *stateInfo) = 0;
    virtual std::unique_ptr<StateQuery> makeNewQuery() = 0;
};
} /* namespace solver */

#endif /* STATESPATIALINDEX_HPP_ */
