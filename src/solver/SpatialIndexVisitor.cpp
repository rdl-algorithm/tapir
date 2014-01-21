#include "SpatialIndexVisitor.hpp"

#include <vector>
#include <spatialindex/SpatialIndex.h>
#include "StatePool.hpp"

namespace solver {

SpatialIndexVisitor::SpatialIndexVisitor(StatePool *statePool) :
        statePool_(statePool) {
}

SpatialIndexVisitor::~SpatialIndexVisitor() {
}

void SpatialIndexVisitor::visitNode(const SpatialIndex::INode &/*node*/) {
}

void SpatialIndexVisitor::visitData(const SpatialIndex::IData &data) {
        visit(statePool_->getStateById(data.getIdentifier()));
}

void SpatialIndexVisitor::visitData(std::vector<const SpatialIndex::IData*> &/*v*/) {
}

} /* namespace solver */
