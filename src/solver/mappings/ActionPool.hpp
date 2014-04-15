#ifndef SOLVER_ACTIONPOOL_HPP_
#define SOLVER_ACTIONPOOL_HPP_

#include "global.hpp"

#include "ActionMapping.hpp"
#include "solver/BeliefNode.hpp"

namespace solver {
class HistoricalData;
class ObservationPool;
class Solver;

class ActionPool {
    friend class Solver;
public:
    ActionPool(Solver *solver) :
        solver_(solver) {
    }
    virtual ~ActionPool() = default;
    _NO_COPY_OR_MOVE(ActionPool);

    /** Creates a default action mapping. */
    virtual std::unique_ptr<ActionMapping> createActionMapping() = 0;

    /** Creates a mapping for the given belief node. */
    virtual void createMappingFor(BeliefNode *node) {
        std::unique_ptr<ActionMapping> map = createActionMapping();
        ActionMapping *mapPtr = map.get();
        node->setMapping(std::move(map));
        mapPtr->initialize();
    }

    /** Returns a rollout action based on the given historical data. */
    virtual std::unique_ptr<Action> getRolloutAction(HistoricalData *data) const = 0;

    /** Returns the solver associated with this pool. */
    virtual Solver * getSolver() {
        return solver_;
    }
private:
    Solver *solver_ = nullptr;
};

} /* namespace solver */

#endif /* SOLVER_ACTIONPOOL_HPP_ */
