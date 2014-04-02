#ifndef SOLVER_BELIEFTREE_HPP_
#define SOLVER_BELIEFTREE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "abstract-problem/Action.hpp"
#include "abstract-problem/Observation.hpp"

#include "global.hpp"

namespace solver {
class ActionMapping;
class BeliefNode;
class Solver;

class BeliefTree {
  public:
    friend class TextSerializer;

    /* Constructs an empty belief tree. */
    BeliefTree(Solver *solver);

    // Default destructor; copying and moving disallowed!
    ~BeliefTree();
    _NO_COPY_OR_MOVE(BeliefTree);

    /* -------------- Node setters and getters ---------------- */
    /** Resets the tree and sets it root to be the given new node. */
    BeliefNode *setRoot(std::unique_ptr<BeliefNode> root);
    /** Returns the root node. */
    BeliefNode *getRoot() const;

    /** Sets the child with the given ID. */
    void setNode(long id, BeliefNode *node);
    /** Retrieves the node with the given ID. */
    BeliefNode *getNode(long id) const;

    /** Returns the number of belief nodes. */
    long getNumberOfNodes() const;
    /** Retrieves a vector of all belief nodes within the policy. */
    std::vector<BeliefNode *> getNodes();

    /* ------------------- Tree modification ------------------- */
    /**
     * Adds a child belief node to the given belief node; this node will be
     * added to the flattened node list if and only if it is actually new.
     */
    BeliefNode *createOrGetChild(BeliefNode *node, Action const &action,
            Observation const &obs);

private:
    Solver *solver_;
    std::unique_ptr<BeliefNode> root_;
    std::vector<BeliefNode *> allNodes_;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFTREE_HPP_ */
