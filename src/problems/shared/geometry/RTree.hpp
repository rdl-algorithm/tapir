/** @file geometry/RTree.hpp
 *
 * Defines the geometry::RTree class, which wraps the R*-tree implementation in
 * libspatialindex.
 */
#ifndef GEOMETRY_RTREE_HPP_
#define GEOMETRY_RTREE_HPP_

#include <memory>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>

#include "global.hpp"

namespace geometry {

/** A wrapper class for the R*-tree implementation in libspatialindex; the main function of this
 * class is to set up the properties and storage manager for the R*-tree.
 */
class RTree {
  public:
    /** Constructs a new RTree with the given number of spatial dimensions. */
    RTree(unsigned int nSDim);
    virtual ~RTree() = default;
    _NO_COPY_OR_MOVE(RTree);

    /** Resets the RTree to be completely empty. */
    virtual void reset();

    /** Returns the ISpatialIndex contained within the tree. */
    SpatialIndex::ISpatialIndex *getTree();

  private:
    /** The number of spatial dimensions for the tree. */
    unsigned int nSDim_;
    /** The set of properties for the tree. */
    std::unique_ptr<Tools::PropertySet> properties_;
    /** The storage manager for the tree. */
    std::unique_ptr<SpatialIndex::IStorageManager> storageManager_;
    /** The actual R*-tree instance itself. */
    std::unique_ptr<SpatialIndex::ISpatialIndex> tree_;
};
} /* namespace geometry */

#endif /* GEOMETRY_RTREE_HPP_ */
