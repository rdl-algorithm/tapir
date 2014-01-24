#include "BeliefTree.hpp"

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "defs.hpp"                     // for make_unique

#include "BeliefNode.hpp"               // for BeliefNode

namespace solver {
BeliefTree::BeliefTree() :
    root_(std::make_unique<BeliefNode>()),
    allNodes_() {
    allNodes_.push_back(root_.get());
}

// Do nothing!
BeliefTree::~BeliefTree() {
}

BeliefNode *BeliefTree::addBeliefNode(BeliefNode *node,
        Action const &action, Observation const &observation) {
    bool isNew;
    BeliefNode *childNode;
    std::tie(childNode, isNew) = node->addChild(action, observation);
    if (isNew) {
        enlistNode(childNode);
    }
    return childNode;
}

// Add a node to the list
void BeliefTree::enlistNode(BeliefNode *node) {
    allNodes_.push_back(node);
}

void BeliefTree::reset() {
    allNodes_.clear();
    root_ = std::make_unique<BeliefNode>();
    allNodes_.push_back(root_.get());
}
} /* namespace solver */
