#ifndef SOLVER_RTREE_HPP_
#define SOLVER_RTREE_HPP_

#include <memory>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>

#include "StateIndex.hpp"

#include "global.hpp"

namespace solver {
class SpatialIndexVisitor;
class StateInfo;
class StatePool;

class RTree : public StateIndex {
  public:
    RTree(unsigned int nSDim);
    virtual ~RTree() = default;
    _NO_COPY_OR_MOVE(RTree);

    virtual void reset() override;
    virtual void addStateInfo(StateInfo *stateInfo) override;
    virtual void removeStateInfo(StateInfo *stateInfo) override;
    virtual void boxQuery(SpatialIndexVisitor &visitor,
            std::vector<double> lowCorner,
            std::vector<double> highCorner);

  private:
    unsigned int nSDim_;
    std::unique_ptr<Tools::PropertySet> properties_;
    std::unique_ptr<SpatialIndex::IStorageManager> storageManager_;
    std::unique_ptr<SpatialIndex::ISpatialIndex> tree_;
};
} /* namespace solver */

#endif /* SOLVER_RTREE_HPP_ */
