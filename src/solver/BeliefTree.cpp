#include "BeliefTree.hpp"

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector
#include <iostream>

#include "global.hpp"                     // for make_unique

#include "abstract-problem/Observation.hpp"

#include "mappings/ActionMapping.hpp"
#include "mappings/ActionPool.hpp"

#include "BeliefNode.hpp"               // for BeliefNode
#include "Solver.hpp"

namespace solver {
BeliefTree::BeliefTree(Solver *solver) :
    solver_(solver),
    root_(nullptr),
    allNodes_() {
}

// Do nothing!
BeliefTree::~BeliefTree() {
}

/* -------------- Node setters and getters ---------------- */
BeliefNode *BeliefTree::setRoot(std::unique_ptr<BeliefNode> root) {
    allNodes_.clear();
    root_ = std::move(root);
    BeliefNode *rootPtr = root_.get();
    allNodes_.push_back(nullptr);
    setNode(0, rootPtr);
    return rootPtr;
}
BeliefNode *BeliefTree::getRoot() const {
    return root_.get();
}

void BeliefTree::setNode(long id, BeliefNode *node) {
    if (allNodes_[id] != nullptr) {
        debug::show_message("ERROR: Node already exists - overwriting!!");
    }
    allNodes_[id] = node;
    node->setId(id);
}
BeliefNode *BeliefTree::getNode(long id) const {
    BeliefNode *node = allNodes_[id];
    if (node->getId() != id) {
        std::ostringstream message;
        message << "ERROR: ID mismatch in Belief Tree - ID should be " << id;
        message << " but was " << node->getId();
        debug::show_message(message.str());;
    }
    return allNodes_[id];
}

long BeliefTree::getNumberOfNodes() const {
    return allNodes_.size();
}
std::vector<BeliefNode *> BeliefTree::getNodes() {
    return allNodes_;
}

/* ------------------- Tree modification ------------------- */
BeliefNode *BeliefTree::createOrGetChild(BeliefNode *node,
        Action const &action, Observation const &obs) {
    bool isNew;
    BeliefNode *childNode;
    std::tie(childNode, isNew) = node->createOrGetChild(action, obs);
    if (isNew) {
        HistoricalData *data = node->getHistoricalData();
        if (data != nullptr) {
                childNode->setHistoricalData(data->createChild(action, obs));
        }
        solver_->getActionPool()->createMappingFor(childNode);
        allNodes_.push_back(nullptr);
        setNode(allNodes_.size() - 1, childNode);
    }
    return childNode;
}
} /* namespace solver */
