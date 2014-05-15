#ifndef SOLVER_ESTIMATION_HPP_
#define SOLVER_ESTIMATION_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {
class BeliefQValueEstimator;

class BeliefEstimationStrategy {
public:
    BeliefEstimationStrategy() = default;
    virtual ~BeliefEstimationStrategy() = default;

    virtual std::unique_ptr<BeliefQValueEstimator> createEstimator(ActionMapping *mapping) = 0;
};

class BeliefQValueEstimator {
public:
    BeliefQValueEstimator() = default;
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

#endif /* SOLVER_ESTIMATION_HPP_ */
