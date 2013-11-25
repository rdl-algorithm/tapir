#ifndef BELIEFTREE_H
#define BELIEFTREE_H

#include <vector>
#include <queue>
#include <fstream>

#include "BeliefNode.h"
#include "Model.h"
#include "Histories.h"

class BeliefTree {
public:
    friend class Solver;

    BeliefTree();
    ~BeliefTree();
    void deleteHelper(std::queue<BeliefNode*> &tmpNodes);
    BeliefTree(const BeliefTree&) = delete;
    BeliefTree(BeliefTree&) = delete;
    BeliefTree &operator=(const BeliefTree&) = delete;
    BeliefTree &operator=(BeliefTree&) = delete;

    void readPolicy(std::ifstream &inFile, Histories *allHist);
    void write(std::ostream &os);
    void writeHelp(std::queue<BeliefNode*> &tmpNodes, std::ostream &os);

    BeliefNode* getRoot() {
        return root;
    }

private:
    BeliefNode *root;
    std::vector<BeliefNode*> allNodes;
};
#endif
