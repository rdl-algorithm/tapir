#ifndef SOLVER_HISTORYCORRECTOR_HPP_
#define SOLVER_HISTORYCORRECTOR_HPP_

#include <unordered_set>

#include "global.hpp"

namespace solver {
class HistorySequence;
class Model;
class Solver;

class HistoryCorrector {
public:
    HistoryCorrector(Solver *solver, Model *model) :
        solver_(solver),
        model_(model) {
    }
    virtual ~HistoryCorrector() = default;
    _NO_COPY_OR_MOVE(HistoryCorrector);

    virtual void setSolver(Solver *solver) {
        solver_ = solver;
    }

    virtual void reviseHistories(
            std::unordered_set<HistorySequence *> &affectedSequences) {
        for (HistorySequence *sequence : affectedSequences) {
            reviseSequence(sequence);
        }
    }
    virtual void reviseSequence(HistorySequence *sequence) = 0;

protected:
    Solver *solver_;
    Model *model_;
};

} /* namespace solver */

#endif /* SOLVER_HISTORYCORRECTOR_HPP_ */
