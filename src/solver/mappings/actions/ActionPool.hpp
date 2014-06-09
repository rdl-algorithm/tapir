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
    /** Creates an action mapping for the given belief node. */
    virtual std::unique_ptr<ActionMapping> createActionMapping(BeliefNode *node) = 0;
};
} /* namespace solver */

#endif /* SOLVER_ACTION_MAPPING_INTERFACE_HPP_ */
