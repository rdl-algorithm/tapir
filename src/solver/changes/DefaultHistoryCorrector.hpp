#ifndef SOLVER_DEFAULTHISTORYCORRECTOR_HPP_
#define SOLVER_DEFAULTHISTORYCORRECTOR_HPP_

#include "HistoryCorrector.hpp"

#include "solver/abstract-problem/heuristics/Heuristic.hpp"

namespace solver {
class HistorySequence;

class DefaultHistoryCorrector: public solver::HistoryCorrector {
public:
    DefaultHistoryCorrector(Solver *solver, Heuristic heuristic);
    virtual ~DefaultHistoryCorrector() = default;
    _NO_COPY_OR_MOVE(DefaultHistoryCorrector);

    virtual bool reviseSequence(HistorySequence *sequence) override;
private:
    Heuristic heuristic_;
};

} /* namespace solver */

#endif /* SOLVER_DEFAULTHISTORYCORRECTOR_HPP_ */
