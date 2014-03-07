#include "FlaggingVisitor.hpp"

#include "solver/changes/ChangeFlags.hpp"
#include "solver/StatePool.hpp"

namespace solver {

FlaggingVisitor::FlaggingVisitor(StatePool *pool,
        ChangeFlags flagsToSet) :
                SpatialIndexVisitor(pool),
                flagsToSet_(flagsToSet) {
}

void FlaggingVisitor::visit(StateInfo* info) {
    statePool_->setChangeFlags(info, flagsToSet_);
}

} /* namespace solver */
