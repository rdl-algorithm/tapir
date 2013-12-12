#ifndef SOLVER_BELIEFTREE_HPP_
#define SOLVER_BELIEFTREE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

namespace solver {
class BeliefNode;

class BeliefTree {
  public:
    friend class Solver;
    friend class TextSerializer;

    /* Constructs a belief tree with only a root. */
    BeliefTree();
    /** Default destructor. */
    ~BeliefTree();

    /* Copying and moving is disallowed. */
    BeliefTree(BeliefTree const &) = delete;
    BeliefTree(BeliefTree &&) = delete;
    BeliefTree &operator=(BeliefTree const &) = delete;
    BeliefTree &operator=(BeliefTree &&) = delete;

    /** Resets the tree to be empty. */
    void reset();

    /** Returns the root node. */
    BeliefNode *getRoot() {
        return root_.get();
    }

    /** Adds a node to the flattened list of nodes. */
    void enlistNode(BeliefNode *node);

  private:
    std::unique_ptr<BeliefNode> root_;
    std::vector<BeliefNode *> allNodes_;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFTREE_HPP_ */
