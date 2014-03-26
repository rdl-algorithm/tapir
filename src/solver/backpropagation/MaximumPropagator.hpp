#ifndef SOLVER_MAXIMUMPROPAGATOR_HPP_
#define SOLVER_MAXIMUMPROPAGATOR_HPP_

#include "BackpropagationStrategy.hpp"

namespace solver {

class MaximumPropagator : public AbstractBackpropagationStrategy {
public:
    MaximumPropagator(Solver *solver);
    virtual ~MaximumPropagator() = default;

    virtual void updateEnd(HistoryEntry *entry, bool undo) override;
    virtual void updateEntry(HistoryEntry *entry, bool undo) override;
    virtual void updateRoot(HistoryEntry *entry, bool undo) override;
};

} /* namespace solver */

#endif /* SOLVER_MAXIMUMPROPAGATOR_HPP_ */
