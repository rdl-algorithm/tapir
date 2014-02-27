#ifndef GEOMETRY_RTREE_HPP_
#define GEOMETRY_RTREE_HPP_

#include <memory>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>

#include "global.hpp"

namespace geometry {

class RTree {
  public:
    RTree(unsigned int nSDim);
    virtual ~RTree() = default;
    _NO_COPY_OR_MOVE(RTree);

    virtual void reset();

    SpatialIndex::ISpatialIndex *getTree();

  private:
    unsigned int nSDim_;
    std::unique_ptr<Tools::PropertySet> properties_;
    std::unique_ptr<SpatialIndex::IStorageManager> storageManager_;
    std::unique_ptr<SpatialIndex::ISpatialIndex> tree_;
};
} /* namespace geometry */

#endif /* GEOMETRY_RTREE_HPP_ */
