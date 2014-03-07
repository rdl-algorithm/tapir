#ifndef SOLVER_ACTIONMAPPING_HPP_
#define SOLVER_ACTIONMAPPING_HPP_

#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Action.hpp"              // for Action
#include "global.hpp"

namespace solver {
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

    /** Returns the number of distinct action nodes used within this mapping. */
    virtual long getNChildren() const = 0;

    /** Returns true iff there is another action that needs to be explored. */
    virtual bool hasActionToTry() const = 0;
    /** Returns the next action to attempt, if any. */
    virtual std::unique_ptr<Action> getNextActionToTry() = 0;

    /** Chooses a next action with the UCB algorithm. */
    virtual std::unique_ptr<Action> getSearchAction(
            double exploreCofficient) = 0;

    /** Updates the calculation of which action is optimal. */
    virtual void updateBestValue() = 0;
    /** Chooses the action with the best expected value */
    virtual std::unique_ptr<Action> getBestAction() const = 0;
    /** Returns the best q-value */
    virtual double getBestMeanQValue() const = 0;


    virtual std::vector<ActionNode *> getChildren() const = 0;
};
} /* namespace solver */

#endif /* SOLVER_ACTIONMAPPING_HPP_ */
