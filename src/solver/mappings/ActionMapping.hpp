#ifndef SOLVER_ACTIONMAPPING_HPP_
#define SOLVER_ACTIONMAPPING_HPP_

#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Action.hpp"              // for Action
#include "global.hpp"

namespace solver {
class ActionMappingEntry;
class ActionNode;

class ActionMapping {
public:
    /** Creates an empty observation map */
    ActionMapping() = default;

    // Default destructor; copying and moving disallowed!
    virtual ~ActionMapping() = default;
    _NO_COPY_OR_MOVE(ActionMapping);

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
    /** Adds or removes visits associated with the given action. */
    virtual void updateVisitCount(Action const &action, long deltaNVisits) = 0;
    /** Updates the q-value associated with a given action, by adding the
     * given amount to the total.
     */
    virtual void updateTotalQValue(Action const &action, double deltaQ)= 0;
    /** Updates the optimal q-value. */
    virtual void update() = 0;
};

class ActionMappingEntry {
public:
    ActionMappingEntry() = default;
    virtual ~ActionMappingEntry() = default;
    _NO_COPY_OR_MOVE(ActionMappingEntry);

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
