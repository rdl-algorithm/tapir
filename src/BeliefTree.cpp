#include "BeliefTree.hpp"

#include <memory>                       // for unique_ptr
#include <queue>                        // for queue
#include <vector>                       // for vector

#include "defs.hpp"                     // for make_unique
#include "ActionNode.hpp"               // for ActionNode
#include "BeliefNode.hpp"               // for BeliefNode

BeliefTree::BeliefTree() :
    root(std::make_unique<BeliefNode>()),
    allNodes() {
    allNodes.push_back(root.get());
}

// Do nothing!
BeliefTree::~BeliefTree() {
}

void BeliefTree::reset() {
    allNodes.clear();
    root = std::make_unique<BeliefNode>();
    allNodes.push_back(root.get());
}
