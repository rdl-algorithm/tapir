#include "SpatialIndexVisitor.hpp"

#include <vector>
#include <spatialindex/SpatialIndex.h>
#include "solver/StatePool.hpp"

namespace solver {

SpatialIndexVisitor::SpatialIndexVisitor(StatePool *statePool) :
        statePool_(statePool) {
}

SpatialIndexVisitor::~SpatialIndexVisitor() {
}

void SpatialIndexVisitor::visitNode(const SpatialIndex::INode &/*node*/) {
}

void SpatialIndexVisitor::visitData(const SpatialIndex::IData &data) {
        visit(statePool_->getInfoById(data.getIdentifier()));
}

void SpatialIndexVisitor::visitData(std::vector<const SpatialIndex::IData*> &/*v*/) {
}

StatePool *SpatialIndexVisitor::getStatePool() const {
    return statePool_;
}

} /* namespace solver */
