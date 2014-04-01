#ifndef SOLVER_ACTIONPOOL_HPP_
#define SOLVER_ACTIONPOOL_HPP_

#include "global.hpp"

#include "ActionMapping.hpp"

namespace solver {
class BeliefNode;
class ObservationPool;

class ActionPool {
    friend class Solver;
public:
    ActionPool() = default;
    virtual ~ActionPool() = default;
    _NO_COPY_OR_MOVE(ActionPool);

    /** Creates a default action mapping. */
    virtual std::unique_ptr<ActionMapping> createActionMapping() = 0;

    /** Creates a default action mapping, then initializes it based on the
     * given belief node.
     */
    virtual std::unique_ptr<ActionMapping> createActionMapping(BeliefNode *node) {
        std::unique_ptr<ActionMapping> map = createActionMapping();
        map->initialize(node);
        return map;
    }
protected:
    ObservationPool *observationPool_ = nullptr;
};

} /* namespace solver */

#endif /* SOLVER_ACTIONPOOL_HPP_ */
