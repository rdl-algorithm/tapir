#ifndef SOLVER_BELIEFTREE_HPP_
#define SOLVER_BELIEFTREE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "abstract-problem/Action.hpp"
#include "abstract-problem/Observation.hpp"

#include "global.hpp"

namespace solver {
class ActionMapping;
class AbstractSearchInstance;
class BeliefNode;
class Solver;

class BeliefTree {
    friend class AbstractSearchInstance;
    friend class Agent;
    friend class HistorySequence;
    friend class Solver;
    friend class TextSerializer;

public:
    /* Constructs an empty belief tree. */
    BeliefTree(Solver *solver);

    // Default destructor; copying and moving disallowed!
    ~BeliefTree();
    _NO_COPY_OR_MOVE(BeliefTree);

    /* ------------------- Simple getters --------------------- */
    /** Returns the root node. */
    BeliefNode *getRoot() const;
    /** Retrieves the node with the given ID. */
    BeliefNode *getNode(long id) const;
    /** Returns the number of belief nodes. */
    long getNumberOfNodes() const;
    /** Retrieves a vector of all belief nodes within the policy. */
    std::vector<BeliefNode *> getNodes() const;


private:
    /* ------------------- Node index modification ------------------- */
    /** Adds the given node to the index of nodes. */
    void addNode(BeliefNode *node);

    /* ------------------- Tree modification ------------------- */
    /** Resets the tree, creating a new root node and returning it. */
    BeliefNode *reset();
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
