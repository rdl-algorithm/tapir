#ifndef solver_FLAGGINGVISITOR_HPP_
#define solver_FLAGGINGVISITOR_HPP_

#include "SpatialIndexVisitor.hpp"
#include "ChangeFlags.hpp"

namespace solver {
class StateInfo;
class StatePool;

class FlaggingVisitor: public solver::SpatialIndexVisitor {
  public:
    FlaggingVisitor(StatePool *pool, ChangeFlags flagsToSet);
    virtual ~FlaggingVisitor() = default;

    virtual void visit(StateInfo *info);
  private:
    ChangeFlags flagsToSet_;
};

} /* namespace solver */

#endif /* solver_FLAGGINGVISITOR_HPP_ */
