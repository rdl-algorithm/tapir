#include <memory>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>
#include <spatialindex/tools/Tools.h>

#include "defs.hpp"
#include "RTree.hpp"
#include "SpatialIndexQuery.hpp"
#include "State.hpp"
#include "StateInfo.hpp"
#include "VectorState.hpp"

namespace solver {
RTree::RTree(unsigned long nDim, StatePool *statePool) :
        nDim_(nDim),
        statePool_(statePool),
        storageManager_(nullptr),
        tree_(nullptr) {
    storageManager_.reset(
            SpatialIndex::StorageManager::createNewMemoryStorageManager());
    Tools::PropertySet properties;
    Tools::Variant var;
    var.m_varType = Tools::VT_ULONG;
    var.m_val.ulVal = nDim;
    properties.setProperty("Dimension", var);

    var.m_varType = Tools::VT_ULONG;
    var.m_val.ulVal = 100;
    properties.setProperty("IndexCapacity", var);

    var.m_varType = Tools::VT_ULONG;
    var.m_val.ulVal = 100;
    properties.setProperty("LeafCapacity", var);

    var.m_varType = Tools::VT_DOUBLE;
    var.m_val.dblVal = 0.7;
    properties.setProperty("FillFactor", var);

    var.m_varType = Tools::VT_LONG;
    var.m_val.lVal = SpatialIndex::RTree::RV_RSTAR;
    properties.setProperty("TreeVariant", var);

    tree_.reset(SpatialIndex::RTree::returnRTree(*storageManager_, properties));
}

void RTree::addStateInfo(StateInfo *stateInfo) {
    SpatialIndex::id_type stateId = stateInfo->getId();
    std::vector<double> vectorData = static_cast<VectorState*>(stateInfo->getState())->asVector();
    SpatialIndex::Point point(&vectorData[0], nDim_);
    tree_->insertData(0, nullptr, point, stateId);
}

void RTree::removeStateInfo(StateInfo *stateInfo) {
    SpatialIndex::id_type stateId = stateInfo->getId();
    std::vector<double> vectorData = static_cast<VectorState*>(stateInfo->getState())->asVector();
    SpatialIndex::Point point(&vectorData[0], nDim_);
    tree_->deleteData(point, stateId);
}

std::unique_ptr<StateQuery> RTree::makeNewQuery() {
    return std::make_unique<SpatialIndexQuery>(nDim_, statePool_, tree_.get());
}

} /* namespace solver */
