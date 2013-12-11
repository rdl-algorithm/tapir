#ifndef BELIEFTREE_HPP
#define BELIEFTREE_HPP

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

class BeliefNode;

class BeliefTree {
  public:
    friend class Solver;
    friend class TextSerializer;

    /* Constructs a belief tree with only a root. */
    BeliefTree();
    /** Default destructor. */
    ~BeliefTree();
    BeliefTree(BeliefTree const &) = delete;
    BeliefTree(BeliefTree &&) = delete;
    BeliefTree &operator=(BeliefTree const &) = delete;
    BeliefTree &operator=(BeliefTree &&) = delete;

    void reset();

    /** Returns the root node. */
    BeliefNode *getRoot() {
        return root.get();
    }

    /** Adds a node to the flattened list of nodes. */
    void enlistNode(BeliefNode *node);

  private:
    std::unique_ptr<BeliefNode> root;
    std::vector<BeliefNode *> allNodes;
};

#endif /* BELIEFTREE_HPP */
