#include "FlaggingVisitor.hpp"

#include "ChangeFlags.hpp"
#include "StatePool.hpp"

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
