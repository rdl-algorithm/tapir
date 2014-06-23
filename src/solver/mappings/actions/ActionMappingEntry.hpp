#ifndef SOLVER_ACTIONMAPPINGENTRY_HPP_
#define SOLVER_ACTIONMAPPINGENTRY_HPP_

#include "solver/abstract-problem/Action.hpp"

namespace solver {
class ActionNode;
class ActionMapping;

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
    /** Returns true iff this action is legal (illegal => totally ignored) */
    virtual bool isLegal() const = 0;

    /** Updates this action, by adding the given number of visits and the
     * given change in the total q-value.
     *
     * This version of the method also updates the last change time of the
     * belief node, so that cached values will be updated for this
     * belief next time they are needed.
     *
     * Returns true if and only if the q value of the action changed.
     */
    virtual bool update(long deltaNVisits, double deltaTotalQ);

    /** Updates this action, by adding the given number of visits and the
     * given change in the total q-value.
     *
     * Returns true if and only if the q value of the action changed.
     */
    virtual bool updateValue(long deltaNVisits, double deltaTotalQ) = 0;
    /** Sets the legality of this action; this can be combined with model changes to force
     * replanning instead of taking illegal actions.
     */
    virtual void setLegal(bool legal) = 0;
};
} /* namespace solver */

#endif /* SOLVER_ACTIONMAPPINGENTRY_HPP_ */
