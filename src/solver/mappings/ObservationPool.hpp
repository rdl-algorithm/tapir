#ifndef SOLVER_OBSERVATIONPOOL_HPP_
#define SOLVER_OBSERVATIONPOOL_HPP_

#include "global.hpp"

namespace solver {
class ActionPool;
class ObservationMapping;
class Solver;

class ObservationPool {
public:
    ObservationPool() = default;
    virtual ~ObservationPool() = default;
    /** Creates a default observation mapping. */
    virtual std::unique_ptr<ObservationMapping> createObservationMapping() = 0;
};

} /* namespace solver */

#endif /* SOLVER_OBSERVATIONPOOL_HPP_ */
