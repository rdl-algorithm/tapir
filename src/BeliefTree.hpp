#ifndef BELIEFTREE_HPP
#define BELIEFTREE_HPP

#include <vector>                       // for vector
class BeliefNode;

class BeliefTree {
  public:
    friend class Solver;
    friend class TextSerializer;

    BeliefTree();
    ~BeliefTree();
    BeliefTree(BeliefTree const &) = delete;
    BeliefTree(BeliefTree &&) = delete;
    BeliefTree &operator=(BeliefTree const &) = delete;
    BeliefTree &operator=(BeliefTree &&) = delete;
    void reset();

    BeliefNode *getRoot() {
        return root;
    }

  private:
    BeliefNode *root;
    std::vector<BeliefNode *> allNodes;
};

#endif /* BELIEFTREE_HPP */
