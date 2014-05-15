#ifndef SOLVER_BELIEFQVALUEESTIMATOR_HPP_
#define SOLVER_BELIEFQVALUEESTIMATOR_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

namespace solver {

class BeliefQValueEstimator {
public:
    BeliefQValueEstimator(ActionMapping *mapping) = default;
    virtual ~BeliefQValueEstimator() = default;

    /** Makes an update happen.
     *
     * Changing the values of the actions will not trigger an update,
     * which allows the change in the q-value to be calculated by
     * retrieving the belief q-value before and after a recalculation.
     */
    virtual void recalculate() = 0;

    /** Returns the last calculated q-value for the belief - this
     * is determined at the last time recalculate() was called.
     */
    virtual double getBeliefQValue() const = 0;

    /** Returns the current action recommendation. */
    virtual std::unique_ptr<Action> getRecommendedAction() const = 0;
};
} /* namespace solver */

#endif /* SOLVER_BELIEFQVALUEESTIMATOR_HPP_ */
