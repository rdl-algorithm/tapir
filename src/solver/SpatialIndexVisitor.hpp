#ifndef SPATIALINDEXVISITOR_HPP_
#define SPATIALINDEXVISITOR_HPP_

#include <vector>

#include <spatialindex/SpatialIndex.h>

namespace solver {
class StateInfo;
class StatePool;

class SpatialIndexVisitor: public SpatialIndex::IVisitor {
  public:
    SpatialIndexVisitor(StatePool *statePool);
    virtual ~SpatialIndexVisitor();

    SpatialIndexVisitor(SpatialIndexVisitor const &) = delete;
    SpatialIndexVisitor(SpatialIndexVisitor &&) = delete;
    SpatialIndexVisitor &operator=(SpatialIndexVisitor const &) = delete;
    SpatialIndexVisitor &operator=(SpatialIndexVisitor &&) = delete;

    virtual void visitNode(const SpatialIndex::INode &node);
    virtual void visitData(const SpatialIndex::IData &data);
    virtual void visitData(std::vector<const SpatialIndex::IData*> &v);
    virtual void visit(StateInfo *info) = 0;
  protected:
    StatePool *statePool_;
};

} /* namespace solver */

#endif /* SPATIALINDEXVISITOR_HPP_ */
