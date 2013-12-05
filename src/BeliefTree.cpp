#include "BeliefTree.hpp"

#include <queue>                        // for queue
#include <vector>                       // for vector

#include "ActionNode.hpp"               // for ActionNode
#include "BeliefNode.hpp"               // for BeliefNode

BeliefTree::BeliefTree() :
    root(new BeliefNode()),
    allNodes() {
    allNodes.push_back(root);
}

BeliefTree::~BeliefTree() {
    reset();
    delete root;
}

void BeliefTree::reset() {
    std::queue<BeliefNode *> tmpNodes;
    tmpNodes.push(root);
    while (!tmpNodes.empty()) {
        BeliefNode *node = tmpNodes.front();
        tmpNodes.pop();
        node->enqueueChildren(tmpNodes);
        delete node;
    }
    allNodes.clear();
    root = std::make_unique<BeliefNode>();
    allNodes.push_back(root);
}
