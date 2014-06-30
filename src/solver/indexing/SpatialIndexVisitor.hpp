/** file: SpatialIndexVisitor.hpp
 *
 * Defines the SpatialIndexVisitor class, which provides a visitor pattern-based approach to
 * deal with the results of querying states within an RTree.
 */
#ifndef SOLVER_SPATIALINDEXVISITOR_HPP_
#define SOLVER_SPATIALINDEXVISITOR_HPP_

#include <vector>

#include <spatialindex/SpatialIndex.h>

#include "global.hpp"

namespace solver {
class StateInfo;
class StatePool;

/** An abstract class which implements the IVisitor interface from libspatialindex in order
 * to provide a visitor pattern approach to querying states within an RTree.
 *
 * The mandatory virtual method is the method visit(), which allows the visitor pattern to
 * handle each state visited during the query.
 */
class SpatialIndexVisitor: public SpatialIndex::IVisitor {
  public:
    SpatialIndexVisitor(StatePool *statePool);
    virtual ~SpatialIndexVisitor();
    _NO_COPY_OR_MOVE(SpatialIndexVisitor);

    virtual void visitNode(const SpatialIndex::INode &node);
    virtual void visitData(const SpatialIndex::IData &data);

    /** This is the key method we implement in order to allow queries over StateInfo. */
    virtual void visitData(std::vector<const SpatialIndex::IData*> &v);

    /** A method following the visitor design patter, allowing each StateInfo to be processed. */
    virtual void visit(StateInfo *info) = 0;

    /** Returns the StatePool associated with this SpatialIndexVisitor. */
    StatePool *getStatePool() const;

  private:
    /** The StatePool used by this visitor. */
    StatePool *statePool_;
};

} /* namespace solver */

#endif /* SOLVER_SPATIALINDEXVISITOR_HPP_ */
