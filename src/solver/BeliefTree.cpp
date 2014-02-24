#include "BeliefTree.hpp"

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector
#include <iostream>

#include "global.hpp"                     // for make_unique

#include "geometry/Observation.hpp"
#include "mappings/ActionMapping.hpp"

#include "BeliefNode.hpp"               // for BeliefNode

namespace solver {
BeliefTree::BeliefTree() :
    root_(nullptr),
    allNodes_() {
}

// Do nothing!
BeliefTree::~BeliefTree() {
}

BeliefNode *BeliefTree::setRoot(std::unique_ptr<BeliefNode> root) {
    allNodes_.clear();
    root_ = std::move(root);
    BeliefNode *rootPtr = root_.get();
    allNodes_.push_back(rootPtr);
    return rootPtr;
}

BeliefNode *BeliefTree::getRoot() const {
    return root_.get();
}

BeliefNode *BeliefTree::createOrGetChild(BeliefNode *node,
        Action const &action, Observation const &obs) {
    bool isNew;
    BeliefNode *childNode;
    std::tie(childNode, isNew) = node->createOrGetChild(action, obs);
    if (isNew) {
        allNodes_.push_back(childNode);
    }
    return childNode;
}
} /* namespace solver */
