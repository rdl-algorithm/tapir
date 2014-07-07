/** @file geometry/RTree.cpp
 *
 * Contains the implementation for the geometry::RTree class.
 */
#include "problems/shared/geometry/RTree.hpp"

#include <memory>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>
#include <spatialindex/tools/Tools.h>

#include "global.hpp"

namespace geometry {
RTree::RTree(unsigned int nSDim) :
        nSDim_(nSDim),
        properties_(nullptr),
        storageManager_(nullptr),
        tree_(nullptr) {
    reset();
}

void RTree::reset() {
    properties_ = std::make_unique<Tools::PropertySet>();
    Tools::Variant var;

    var.m_varType = Tools::VT_ULONG;
    var.m_val.ulVal = nSDim_;
    properties_->setProperty("Dimension", var);

    var.m_varType = Tools::VT_ULONG;
    var.m_val.ulVal = 100;
    properties_->setProperty("IndexCapacity", var);

    var.m_varType = Tools::VT_ULONG;
    var.m_val.ulVal = 100;
    properties_->setProperty("LeafCapacity", var);

    var.m_varType = Tools::VT_DOUBLE;
    var.m_val.dblVal = 0.7;
    properties_->setProperty("FillFactor", var);

    var.m_varType = Tools::VT_LONG;
    var.m_val.lVal = SpatialIndex::RTree::RV_RSTAR;
    properties_->setProperty("TreeVariant", var);

    tree_.reset(nullptr);
    storageManager_.reset(
            SpatialIndex::StorageManager::returnMemoryStorageManager(*properties_));
    tree_.reset(SpatialIndex::RTree::returnRTree(*storageManager_, *properties_));
}

SpatialIndex::ISpatialIndex *RTree::getTree() {
    return tree_.get();
}

} /* namespace geometry */
