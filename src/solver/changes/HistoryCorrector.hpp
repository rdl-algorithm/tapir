#ifndef SOLVER_HISTORYCORRECTOR_HPP_
#define SOLVER_HISTORYCORRECTOR_HPP_

#include <unordered_set>

#include "global.hpp"

#include "solver/Solver.hpp"

namespace solver {
class HistorySequence;
class Model;

class HistoryCorrector {
public:
    HistoryCorrector(Solver *solver) :
        solver_(solver) {
    }
    virtual ~HistoryCorrector() = default;
    _NO_COPY_OR_MOVE(HistoryCorrector);

    virtual void reviseHistories(
            std::unordered_set<HistorySequence *> &affectedSequences) {
        for (HistorySequence *sequence : affectedSequences) {
            reviseSequence(sequence);
        }
    }
    virtual void reviseSequence(HistorySequence *sequence) = 0;

    /** Returns the solver used with this corrector. */
    virtual Solver *getSolver() const {
        return solver_;
    }
    /** Returns the model used with this corrector. */
    virtual Model *getModel() const {
        return solver_->getModel();
    }

private:
    Solver *solver_;
};

} /* namespace solver */

#endif /* SOLVER_HISTORYCORRECTOR_HPP_ */
