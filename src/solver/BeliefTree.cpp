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
    allNodes_.push_back(nullptr);
    setNode(0, rootPtr);
    return rootPtr;
}

long BeliefTree::getNumberOfNodes() const {
    return allNodes_.size();
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
        allNodes_.push_back(nullptr);
        setNode(allNodes_.size() - 1, childNode);
    }
    return childNode;
}

BeliefNode *BeliefTree::getNode(long id) {
    BeliefNode *node = allNodes_[id];
    if (node->getId() != id) {
        std::ostringstream message;
        message << "ERROR: ID mismatch in Belief Tree - ID should be " << id;
        message << " but was " << node->getId();
        debug::show_message(message.str());;
    }
    return allNodes_[id];
}

void BeliefTree::setNode(long id, BeliefNode *node) {
    if (allNodes_[id] != nullptr) {
        debug::show_message("ERROR: Node already exists - overwriting!!");
    }
    allNodes_[id] = node;
    node->setId(id);
}
} /* namespace solver */
