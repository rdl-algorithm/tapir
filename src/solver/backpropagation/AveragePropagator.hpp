#ifndef SOLVER_AVERAGEPROPAGATOR_HPP_
#define SOLVER_AVERAGEPROPAGATOR_HPP_

#include "BackpropagationStrategy.hpp"

namespace solver {

class AveragePropagator : public AbstractBackpropagationStrategy {
public:
    AveragePropagator(Solver *solver);
    virtual ~AveragePropagator() = default;
    _NO_COPY_OR_MOVE(AveragePropagator);

    virtual void updateEntry(HistoryEntry *entry, bool undo) override;
};

} /* namespace solver */

#endif /* SOLVER_AVERAGEPROPAGATOR_HPP_ */
