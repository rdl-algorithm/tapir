#ifndef RTREE_HPP_
#define RTREE_HPP_

#include <memory>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>

#include "StateQuery.hpp"
#include "StateSpatialIndex.hpp"


namespace solver {
class StateInfo;
class StatePool;

class RTree : public StateSpatialIndex {
  public:
    RTree(unsigned long nDim, StatePool *statePool);
    virtual ~RTree() = default;
    RTree(RTree const &) = delete;
    RTree(RTree &&) = delete;
    virtual RTree &operator=(RTree const &) = delete;
    virtual RTree &operator=(RTree &&) = delete;

    virtual void addStateInfo(StateInfo *stateInfo);
    virtual void removeStateInfo(StateInfo *stateInfo);
    virtual StateInfo *getNearestNeighbor(StateInfo *stateInfo) = 0;
    virtual std::unique_ptr<StateQuery> makeNewQuery();
  private:
    unsigned long nDim_;
    StatePool *statePool_;
    std::unique_ptr<SpatialIndex::IStorageManager> storageManager_;
    std::unique_ptr<SpatialIndex::ISpatialIndex> tree_;
};
} /* namespace solver */

#endif /* RTREE_HPP_ */
