#ifndef solver_FLAGGINGVISITOR_HPP_
#define solver_FLAGGINGVISITOR_HPP_

#include "solver/ChangeFlags.hpp"

#include "SpatialIndexVisitor.hpp"

#include "global.hpp"

namespace solver {
class StateInfo;
class StatePool;

class FlaggingVisitor: public solver::SpatialIndexVisitor {
  public:
    FlaggingVisitor(StatePool *pool, ChangeFlags flagsToSet);
    virtual ~FlaggingVisitor() = default;
    _NO_COPY_OR_MOVE(FlaggingVisitor);

    virtual void visit(StateInfo *info) override;
  private:
    ChangeFlags flagsToSet_;
};

} /* namespace solver */

#endif /* solver_FLAGGINGVISITOR_HPP_ */
