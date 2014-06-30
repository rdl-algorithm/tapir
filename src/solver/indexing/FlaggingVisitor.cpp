/** file: FlaggingVisitor.cpp
 *
 * Contains the implementation of the FlaggingVisitor class.
 */
#include "solver/indexing/FlaggingVisitor.hpp"

#include "solver/changes/ChangeFlags.hpp"
#include "solver/StatePool.hpp"

namespace solver {

FlaggingVisitor::FlaggingVisitor(StatePool *pool,
        ChangeFlags flagsToSet) :
                SpatialIndexVisitor(pool),
                flagsToSet_(flagsToSet) {
}

void FlaggingVisitor::visit(StateInfo* info) {
    getStatePool()->setChangeFlags(info, flagsToSet_);
}

} /* namespace solver */
