/** @file BeliefTree.hpp
 *
 * Contains the BeliefTree class, which represents an entire belief tree.
 *
 * Most of the work is done in the individual classes for the mappings and nodes; this class
 * simply owns a root node, and keeps track of a vector of all of the nodes in the entire tree
 * for convenient iteration and serialization.
 */
#ifndef SOLVER_BELIEFTREE_HPP_
#define SOLVER_BELIEFTREE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "global.hpp"

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/Observation.hpp"

namespace solver {
class ActionMapping;
class BasicSearchStrategy;
class BeliefNode;
class Solver;

/** Represents a belief tree, which is the core data structure representing a policy for the
 * ABT solver.
 *
 * This class just owns the root belief node; each individual node in the tree is owned by
 * its parent mapping entry.
 *
 * However, this belief tree also keeps track of an index of nodes, represented by a
 * vector of non-owning pointers to the individual belief nodes.
 * This allows for access by node ID, iteration and serialization.
 */
class BeliefTree {
    friend class Agent;
    friend class HistorySequence;
    friend class Solver;
    friend class TextSerializer;

public:
    /** Constructs an empty belief tree. */
    BeliefTree(Solver *solver);
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

    /* ------------------- Creation of new nodes in the tree ------------------- */
    /**
     * Adds a child belief node to the given belief node; this node will be
     * added to the flattened node list if and only if it is actually new.
     *
     * Use this method with care!
     */
    BeliefNode *createOrGetChild(BeliefNode *node, Action const &action,
            Observation const &obs);

private:
    /* ------------------- Node index modification ------------------- */
    /** Adds the given node to the index of nodes. */
    void addNode(BeliefNode *node);

    /* ------------------- Tree modification ------------------- */
    /** Resets the tree, creating a new root node and returning it. */
    BeliefNode *reset();
    /** Initializes the root node - for creating a new tree from scratch. */
    void initializeRoot();


private:
    /** The ABT solver that owns this tree. */
    Solver *solver_;
    /** The root node for this tree. */
    std::unique_ptr<BeliefNode> root_;
    /** A vector of pointers to the all of the nodes of the tree. */
    std::vector<BeliefNode *> allNodes_;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFTREE_HPP_ */
