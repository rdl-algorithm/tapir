#include <unordered_set>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>

#include "SpatialIndexQuery.hpp"
#include "VectorState.hpp"

namespace solver {

SpatialIndexQuery::SpatialIndexQuery(unsigned long nDim,
        StatePool *statePool,
        SpatialIndex::ISpatialIndex *spatialIndex) :
                nDim_(nDim),
                statePool_(statePool),
                spatialIndex_(spatialIndex),
                states_() {
}

StateInfoSet SpatialIndexQuery::getStates() {
    return states_;
}

void SpatialIndexQuery::markStates(VectorState *lowCorner, VectorState *highCorner) {
    SpatialIndex::Region region(&lowCorner->asVector()[0],
            &highCorner->asVector()[0], nDim_);
    Visitor visitor(this);
    spatialIndex_->containsWhatQuery(region, visitor);
}

void SpatialIndexQuery::clearStates() {
    states_.clear();
}

SpatialIndexQuery::Visitor::Visitor(SpatialIndexQuery *query) :
        query_(query) {
}

void SpatialIndexQuery::Visitor::visitNode(
        const SpatialIndex::INode&  /* node */) {
}

void SpatialIndexQuery::Visitor::visitData(
        const SpatialIndex::IData& data) {
    query_->states_.insert(query_->statePool_->getStateById(data.getIdentifier()));
}

void SpatialIndexQuery::Visitor::visitData(
        std::vector<const SpatialIndex::IData*>& /* v */) {
}

} /* namespace solver */
