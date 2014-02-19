#ifndef SOLVER_BELIEFTREE_HPP_
#define SOLVER_BELIEFTREE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "Action.hpp"
#include "Observation.hpp"

namespace solver {
class ActionMapping;
class BeliefNode;

class BeliefTree {
  public:
    friend class Solver;
    friend class TextSerializer;

    /* Constructs a belief tree with only a root. */
    BeliefTree(std::unique_ptr<ActionMapping> actionMap);

    // Default destructor; copying and moving disallowed!
    ~BeliefTree();
    BeliefTree(BeliefTree const &) = delete;
    BeliefTree(BeliefTree &&) = delete;
    BeliefTree &operator=(BeliefTree const &) = delete;
    BeliefTree &operator=(BeliefTree &&) = delete;

    /** Resets the tree to be empty. */
    void reset();

    /** Returns the root node. */
    BeliefNode *getRoot() const;

    /**
     * Adds a child belief node to the given belief node; this node will be
     * added to the flattened node list if and only if it is actually new.
     */
    BeliefNode *createOrGetChild(BeliefNode *node,
            Action const &action, Observation const &obs);

  private:
    std::unique_ptr<BeliefNode> root_;
    std::vector<BeliefNode *> allNodes_;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFTREE_HPP_ */
