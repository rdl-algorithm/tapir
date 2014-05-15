#ifndef SOLVER_ACTIONMAPPING_HPP_
#define SOLVER_ACTIONMAPPING_HPP_

#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Action.hpp"              // for Action
#include "global.hpp"

namespace solver {
class ActionMappingEntry;
class ActionNode;
class BeliefNode;

class ActionMapping {
public:
    ActionMapping() = default;
    virtual ~ActionMapping() = default;

    /* -------------- Association with a belief node ---------------- */
    /* Associates this mapping with the given belief node. */
    virtual void setOwner(BeliefNode *owner) = 0;
    /** Returns the belief node that owns this mapping. */
    virtual BeliefNode *getOwner() const = 0;
    /** Initializes this mapping (with respect to the belief node that owns it)
     *
     * This is optional; it can allow initialization of mappings based on
     * parameters of the given node, including the history of actions and
     * observations.
     *
     * This method is called when creating a new belief node, but is not
     * called when a belief node is deserialized, as the relevant parameters
     * can simply be stored for serialization.
     */
    virtual void initialize() {}

    /* -------------- Creation and retrieval of nodes. ---------------- */
    /** Retrieves the action node (if any) corresponding to this action. */
    virtual ActionNode *getActionNode(Action const &action) const = 0;
    /** Creates a new action node for the given action. */
    virtual ActionNode *createActionNode(Action const &action) = 0;
    /** Returns the number of child nodes associated with this mapping. */
    virtual long getNChildren() const = 0;

    /* -------------- Retrieval of mapping entries. ---------------- */
    /** Returns the number of entries in this mapping with a nonzero visit
     * count (some of these may not have an associated action node, so this
     * is different to the number of child nodes).
     */
    virtual long getNumberOfVisitedEntries() const = 0;
    /** Returns all of the visited entries in this mapping - some may have
     * null action nodes if the visit counts were initialized to nonzero
     * values.
     */
    virtual std::vector<ActionMappingEntry const *> getVisitedEntries() const = 0;
    /** Returns the mapping entry (if any) associated with the given action. */
    virtual ActionMappingEntry const *getEntry(Action const &action) const = 0;

    /* ------------------ Methods for unvisited actions ------------------- */
    virtual bool hasUnvisitedActions() const = 0;
    /** Returns the unvisited actions (that should be visited) for this node. */
    virtual std::vector<std::unique_ptr<Action>> getUnvisitedActions() const = 0;
    /** Returns a random unvisited action. */
    virtual std::unique_ptr<Action> getRandomUnvisitedAction() const = 0;


    /* -------------- Retrieval of general statistics. ---------------- */
    /** Returns the total number of times children have been visited. */
    virtual long getTotalVisitCount() const = 0;

    /* --------------- Methods for updating the values ----------------- */
    /** Updates the given action, by adding the given number of visits and the
     * given change in the total q-value.
    */
    virtual void update(Action const &action, long deltaNVisits, double deltaQ) = 0;
};

class ActionMappingEntry {
public:
    ActionMappingEntry() = default;
    virtual ~ActionMappingEntry() = default;

    /** Returns the mapping this entry belongs to. */
    virtual ActionMapping *getMapping() const = 0;
    /** Returns the action for this entry. */
    virtual std::unique_ptr<Action> getAction() const = 0;
    /** Returns the action node for this entry. */
    virtual ActionNode *getActionNode() const = 0;
    /** Returns the visit count for this entry. */
    virtual long getVisitCount() const = 0;
    /** Returns the total Q-value for this entry. */
    virtual double getTotalQValue() const = 0;
    /** Returns the mean Q-value for this entry. */
    virtual double getMeanQValue() const = 0;
};
} /* namespace solver */

#endif /* SOLVER_ACTIONMAPPING_HPP_ */
