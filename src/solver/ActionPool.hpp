#ifndef SOLVER_ACTIONPOOL_HPP_
#define SOLVER_ACTIONPOOL_HPP_

#include "global.hpp"

namespace solver {
class ActionMapping;
class BeliefNode;
class ObservationPool;

class ActionPool {
    friend class Solver;
public:
    ActionPool() = default;
    virtual ~ActionPool() = default;
    _NO_COPY_OR_MOVE(ActionPool);

    virtual std::unique_ptr<ActionMapping> createActionMapping();
protected:
    ObservationPool *observationPool_ = nullptr;
};

} /* namespace solver */

#endif /* SOLVER_ACTIONPOOL_HPP_ */
