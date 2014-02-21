#ifndef SOLVER_OBSERVATIONPOOL_HPP_
#define SOLVER_OBSERVATIONPOOL_HPP_

#include "global.hpp"

namespace solver {
class ActionPool;
class ObservationMapping;

class ObservationPool {
    friend class Solver;
public:
    ObservationPool() = default;
    virtual ~ObservationPool() = default;
    _NO_COPY_OR_MOVE(ObservationPool);

    virtual std::unique_ptr<ObservationMapping> createObservationMapping() = 0;
protected:
    ActionPool *actionPool_ = nullptr;
};

} /* namespace solver */

#endif /* SOLVER_OBSERVATIONPOOL_HPP_ */
