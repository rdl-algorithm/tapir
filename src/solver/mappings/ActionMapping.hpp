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
    /** Creates an empty observation map */
    ActionMapping() = default;

    // Default destructor; copying and moving disallowed!
    virtual ~ActionMapping() = default;
    _NO_COPY_OR_MOVE(ActionMapping);

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

    /* -------------- Retrieval of mapping entries. ---------------- */
    /** Returns the number of child nodes associated with this mapping. */
    virtual long getNChildren() const = 0;
    /** Returns all the entries associated with children in this mapping (i.e.
     * containing non-null action nodes).
     */
    virtual std::vector<ActionMappingEntry const *> getChildEntries() const = 0;
    /** Returns the mapping entry associated with the given action. */
    virtual ActionMappingEntry const *getEntry(Action const &action) const = 0;

    /* -------------- Retrieval of general statistics. ---------------- */
    /** Returns the total number of times children have been visited. */
    virtual long getTotalVisitCount() const = 0;
    /** Chooses the action with the highest q-value. */
    virtual std::unique_ptr<Action> getBestAction() const = 0;
    /** Returns the best q-value. */
    virtual double getMaxQValue() const = 0;
    /** Returns the robust action (i.e. the action with the highest
     * visit count (optional).
     */
    virtual std::unique_ptr<Action> getRobustAction() const {
        return nullptr;
    }
    /** Returns the robust q-value (optional) */
    virtual double getRobustQValue() const {
        return -std::numeric_limits<double>::infinity();
    }

    /* ------------ Methods for retrieving unvisited actions -------------- */
    /** Returns true iff there is another action that has not been visited. */
    virtual bool hasUnvisitedActions() const = 0;
    /** Returns the unvisited actions for this node. */
    virtual std::vector<std::unique_ptr<Action>> getUnvisitedActions() const = 0;
    /** Returns a random unvisited action. */
    virtual std::unique_ptr<Action> getRandomUnvisitedAction() const = 0;

    /* ------------ Easy getters for entry values. -------------- */
    /** Returns the number of visits for the given action. */
    virtual long getVisitCount(Action const &action) const = 0;
    /** Returns the total q-value for the given action. */
    virtual double getTotalQValue(Action const &action) const = 0;
    /** Returns the mean q-value for the given action. */
    virtual double getMeanQValue(Action const &action) const = 0;

    /* --------------- Methods for updating the values ----------------- */
    /** Updates the given action, by adding the given number of visits and the
     * given change in the total q-value.
    */
    virtual void update(Action const &action, long deltaNVisits, double deltaQ) = 0;
    /** Recalculates the q-values. */
    virtual void recalculate() = 0;
};

class ActionMappingEntry {
public:
    ActionMappingEntry() = default;
    virtual ~ActionMappingEntry() = default;
    _NO_COPY_OR_MOVE(ActionMappingEntry);

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
