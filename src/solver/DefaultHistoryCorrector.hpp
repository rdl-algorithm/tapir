#ifndef SOLVER_DEFAULTHISTORYCORRECTOR_HPP_
#define SOLVER_DEFAULTHISTORYCORRECTOR_HPP_

#include "HistoryCorrector.hpp"

namespace solver {
class HistorySequence;

class DefaultHistoryCorrector: public solver::HistoryCorrector {
public:
    DefaultHistoryCorrector(Model *model);
    virtual ~DefaultHistoryCorrector() = default;

    void reviseSequence(HistorySequence *sequence);
};

} /* namespace solver */

#endif /* SOLVER_DEFAULTHISTORYCORRECTOR_HPP_ */
