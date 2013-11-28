#include "BeliefTree.hpp"

#include <queue>
using std::queue;
#include <vector>
using std::vector;

#include "BeliefNode.hpp"

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
    queue<BeliefNode*> tmpNodes;
    tmpNodes.push(root);
    while (!tmpNodes.empty()) {
        BeliefNode *node = tmpNodes.front();
        tmpNodes.pop();
        node->enqueueChildren(tmpNodes);
        delete node;
    }
    allNodes.clear();
    root = new BeliefNode();
    allNodes.push_back(root);
}
