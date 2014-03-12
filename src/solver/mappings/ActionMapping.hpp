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

    /** Retrieves the action node (if any) corresponding to this action. */
    virtual ActionNode *getActionNode(Action const &action) const = 0;
    /** Creates a new action node for the given action. */
    virtual ActionNode *createActionNode(Action const &action) = 0;

    /** Returns the number of child nodes associated with this mapping.
     */
    virtual long getNChildren() const = 0;
    /** Returns all the entries associated with children in this mapping (i.e.
     * containing non-null action nodes).
     */
    virtual std::vector<ActionMappingEntry const *> getChildEntries() const = 0;

    /** Returns true iff there is another action that needs to be explored. */
    virtual bool hasRolloutActions() const = 0;
    /** Returns a number of actions that should be attempted for this node. */
    virtual std::vector<std::unique_ptr<Action>> getRolloutActions() const = 0;
    /** Returns a random action to use for a rollout. */
    virtual std::unique_ptr<Action> getRandomRolloutAction() const = 0;

    /** Updates the calculation of which action is optimal. */
    virtual void update() = 0;
    /** Chooses the action with the best expected value */
    virtual std::unique_ptr<Action> getBestAction() const = 0;
    /** Returns the best q-value */
    virtual double getBestMeanQValue() const = 0;
};

class ActionMappingEntry {
  public:
    ActionMappingEntry() = default;
    virtual ~ActionMappingEntry() = default;
    _NO_COPY_OR_MOVE(ActionMappingEntry);

    virtual std::unique_ptr<Action> getAction() const = 0;
    virtual ActionNode *getActionNode() const = 0;
};
} /* namespace solver */

#endif /* SOLVER_ACTIONMAPPING_HPP_ */
