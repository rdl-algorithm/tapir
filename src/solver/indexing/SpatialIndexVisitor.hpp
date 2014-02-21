#ifndef SOLVER_SPATIALINDEXVISITOR_HPP_
#define SOLVER_SPATIALINDEXVISITOR_HPP_

#include <vector>

#include <spatialindex/SpatialIndex.h>

#include "global.hpp"

namespace solver {
class StateInfo;
class StatePool;

class SpatialIndexVisitor: public SpatialIndex::IVisitor {
  public:
    SpatialIndexVisitor(StatePool *statePool);
    virtual ~SpatialIndexVisitor();
    _NO_COPY_OR_MOVE(SpatialIndexVisitor);

    virtual void visitNode(const SpatialIndex::INode &node);
    virtual void visitData(const SpatialIndex::IData &data);
    virtual void visitData(std::vector<const SpatialIndex::IData*> &v);
    virtual void visit(StateInfo *info) = 0;
  protected:
    StatePool *statePool_;
};

} /* namespace solver */

#endif /* SOLVER_SPATIALINDEXVISITOR_HPP_ */
