#include "BeliefTree.hpp"

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector
#include <iostream>

#include "defs.hpp"                     // for make_unique

#include "BeliefNode.hpp"               // for BeliefNode
#include "Observation.hpp"

namespace solver {
BeliefTree::BeliefTree() :
    root_(std::make_unique<BeliefNode>()),
    allNodes_() {
    allNodes_.push_back(root_.get());
}

// Do nothing!
BeliefTree::~BeliefTree() {
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

void BeliefTree::reset() {
    allNodes_.clear();
    root_ = std::make_unique<BeliefNode>();
    allNodes_.push_back(root_.get());
}
} /* namespace solver */
