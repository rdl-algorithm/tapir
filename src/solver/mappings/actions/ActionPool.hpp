#ifndef SOLVER_ACTION_MAPPING_INTERFACE_HPP_
#define SOLVER_ACTION_MAPPING_INTERFACE_HPP_

#include <memory>                       // for unique_ptr

#include "global.hpp"

#include "solver/abstract-problem/Action.hpp"              // for Action
#include "solver/BeliefNode.hpp"

namespace solver {
class ActionMapping;
class ActionNode;
class BeliefNode;
class HistoricalData;
class ObservationPool;
class Solver;

class ActionPool {
public:
    ActionPool() = default;
    virtual ~ActionPool() = default;
    /** Creates a default action mapping. */
    virtual std::unique_ptr<ActionMapping> createActionMapping() = 0;

    /** Returns an action based on a default history-based rollout strategy;
     * an implementation is required if you want to use
     * DefaultRolloutStrategy. */
    virtual std::unique_ptr<Action> getDefaultRolloutAction(HistoricalData */*data*/) const {
        return nullptr;
    }
};
} /* namespace solver */

#endif /* SOLVER_ACTION_MAPPING_INTERFACE_HPP_ */
