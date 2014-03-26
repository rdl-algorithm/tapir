#ifndef SOLVER_ROBUSTPROPAGATOR_HPP_
#define SOLVER_ROBUSTPROPAGATOR_HPP_

#include "BackpropagationStrategy.hpp"

namespace solver {

class RobustPropagator : public AbstractBackpropagationStrategy {
public:
    RobustPropagator(Solver *solver);
    virtual ~RobustPropagator() = default;

    virtual void updateEnd(HistoryEntry *entry, bool undo) override;
    virtual void updateEntry(HistoryEntry *entry, bool undo) override;
    virtual void updateRoot(HistoryEntry *entry, bool undo) override;
};

} /* namespace solver */

#endif /* SOLVER_ROBUSTPROPAGATOR_HPP_ */
