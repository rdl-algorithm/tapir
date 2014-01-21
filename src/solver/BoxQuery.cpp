#include <unordered_set>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>

#include "BoxQuery.hpp"
#include "VectorState.hpp"

namespace solver {

BoxQuery::BoxQuery(unsigned long nSDim,
        StatePool *statePool,
        SpatialIndex::ISpatialIndex *spatialIndex) :
                nSDim_(nSDim),
                statePool_(statePool),
                spatialIndex_(spatialIndex),
                states_() {
}

StateInfoSet BoxQuery::getStates() {
    return states_;
}

void BoxQuery::markStates(std::vector<double> lowCorner,
        std::vector<double> highCorner) {
    SpatialIndex::Region region(&lowCorner[0], &highCorner[0], nSDim_);
    Visitor visitor(this);
    spatialIndex_->containsWhatQuery(region, visitor);
}

void BoxQuery::clearStates() {
    states_.clear();
}

BoxQuery::Visitor::Visitor(BoxQuery *query) :
        query_(query) {
}

void BoxQuery::Visitor::visitNode(
        const SpatialIndex::INode&  /* node */) {
}

void BoxQuery::Visitor::visitData(
        const SpatialIndex::IData& data) {
    query_->states_.insert(query_->statePool_->getStateById(data.getIdentifier()));
}

void BoxQuery::Visitor::visitData(
        std::vector<const SpatialIndex::IData*>& /* v */) {
}

} /* namespace solver */
