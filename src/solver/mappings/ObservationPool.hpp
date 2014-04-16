#ifndef SOLVER_OBSERVATIONPOOL_HPP_
#define SOLVER_OBSERVATIONPOOL_HPP_

#include "global.hpp"

namespace solver {
class ActionPool;
class ObservationMapping;
class Solver;

class ObservationPool {
    friend class Solver;
public:
    ObservationPool(Solver *solver) :
        solver_(solver) {
    }
    virtual ~ObservationPool() = default;
    _NO_COPY_OR_MOVE(ObservationPool);

    /** Creates a default observation mapping. */
    virtual std::unique_ptr<ObservationMapping> createObservationMapping() = 0;

    /** Returns the solver associated with this pool. */
    virtual Solver * getSolver() {
        return solver_;
    }
private:
    Solver *solver_ = nullptr;
};

} /* namespace solver */

#endif /* SOLVER_OBSERVATIONPOOL_HPP_ */
