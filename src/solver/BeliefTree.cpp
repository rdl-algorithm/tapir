#include "BeliefTree.hpp"

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector
#include <iostream>

#include "global.hpp"                     // for make_unique

#include "abstract-problem/Observation.hpp"

#include "mappings/actions/ActionMapping.hpp"
#include "mappings/actions/ActionPool.hpp"

#include "BeliefNode.hpp"               // for BeliefNode
#include "Solver.hpp"

namespace solver {
BeliefTree::BeliefTree(Solver *solver) :
    solver_(solver),
    root_(nullptr),
    allNodes_() {
	reset();
}

// Do nothing!
BeliefTree::~BeliefTree() {
}

/* ------------------- Simple getters --------------------- */
BeliefNode *BeliefTree::getRoot() const {
    return root_.get();
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
std::vector<BeliefNode *> BeliefTree::getNodes() const {
    return allNodes_;
}


/* ============================ PRIVATE ============================ */


/* ------------------- Node index modification ------------------- */
void BeliefTree::addNode(BeliefNode *node) {
    long id = node->id_;
    if (id < 0) {
        // Negative ID => allocate a new one.
        id = allNodes_.size();
        node->id_ = id;
        allNodes_.push_back(nullptr);
    }
    if (allNodes_[id] != nullptr) {
        debug::show_message("ERROR: Node already exists - overwriting!!");
    }
    allNodes_[id] = node;
    BeliefNode *parent = node->getParentBelief();
    if (parent == nullptr) {
        node->depth_ = 0;
    } else {
        node->depth_ = parent->depth_ + 1;
    }
}

/* ------------------- Tree modification ------------------- */
BeliefNode *BeliefTree::reset() {
    allNodes_.clear();
    root_ = std::make_unique<BeliefNode>();
    BeliefNode *rootPtr = root_.get();
    addNode(rootPtr);
    return rootPtr;
}
void BeliefTree::initializeRoot() {
    root_->setHistoricalData(solver_->getModel()->createRootHistoricalData());
    root_->setMapping(solver_->getActionPool()->createActionMapping());
    root_->setEstimator(solver_->getBeliefEstimationStrategy()->createEstimator(root_->actionMap_.get()));
    root_->getMapping()->initialize();
}

BeliefNode *BeliefTree::createOrGetChild(BeliefNode *node,
        Action const &action, Observation const &obs) {
    bool isNew;
    BeliefNode *childNode;
    std::tie(childNode, isNew) = node->createOrGetChild(solver_, action, obs);
    if (isNew) {
        addNode(childNode);
        HistoricalData *data = node->getHistoricalData();
        if (data != nullptr) {
            childNode->setHistoricalData(data->createChild(action, obs));
        }
        // Initialize the child node's action mapping.
        childNode->getMapping()->initialize();
    }
    return childNode;
}
} /* namespace solver */
