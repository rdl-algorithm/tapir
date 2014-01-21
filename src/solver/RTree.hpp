#ifndef RTREE_HPP_
#define RTREE_HPP_

#include <memory>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>

#include "StateIndex.hpp"


namespace solver {
class StateInfo;
class StatePool;
class BoxQuery;

class RTree : public StateIndex {
  public:
    RTree(unsigned long nSDim, StatePool *statePool);
    virtual ~RTree() = default;
    RTree(RTree const &) = delete;
    RTree(RTree &&) = delete;
    virtual RTree &operator=(RTree const &) = delete;
    virtual RTree &operator=(RTree &&) = delete;

    virtual void reset();

    virtual void addStateInfo(StateInfo *stateInfo);
    virtual void removeStateInfo(StateInfo *stateInfo);
    virtual StateInfo *getNearestNeighbor(StateInfo *stateInfo);
    virtual std::unique_ptr<BoxQuery> makeBoxQuery();
  private:
    unsigned long nSDim_;
    StatePool *statePool_;
    std::unique_ptr<Tools::PropertySet> properties_;
    std::unique_ptr<SpatialIndex::IStorageManager> storageManager_;
    std::unique_ptr<SpatialIndex::ISpatialIndex> tree_;
};
} /* namespace solver */

#endif /* RTREE_HPP_ */
