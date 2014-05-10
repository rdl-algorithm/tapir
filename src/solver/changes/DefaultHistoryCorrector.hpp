#ifndef SOLVER_DEFAULTHISTORYCORRECTOR_HPP_
#define SOLVER_DEFAULTHISTORYCORRECTOR_HPP_

#include "HistoryCorrector.hpp"

namespace solver {
class HistorySequence;

class DefaultHistoryCorrector: public solver::HistoryCorrector {
public:
    DefaultHistoryCorrector(Solver *solver);
    virtual ~DefaultHistoryCorrector() = default;
    _NO_COPY_OR_MOVE(DefaultHistoryCorrector);

    void reviseSequence(HistorySequence *sequence);
};

} /* namespace solver */

#endif /* SOLVER_DEFAULTHISTORYCORRECTOR_HPP_ */
