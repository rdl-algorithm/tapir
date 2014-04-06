#ifndef SOLVER_MAXIMUMPROPAGATOR_HPP_
#define SOLVER_MAXIMUMPROPAGATOR_HPP_

#include "BackpropagationStrategy.hpp"

namespace solver {

class MaximumPropagator : public AbstractBackpropagationStrategy {
public:
    MaximumPropagator(Solver *solver);
    virtual ~MaximumPropagator() = default;
    _NO_COPY_OR_MOVE(MaximumPropagator);

    virtual void updateEntry(HistoryEntry *entry, bool undo) override;
private:
    double deltaQ_;
    bool isSecondLastEntry_;
};

} /* namespace solver */

#endif /* SOLVER_MAXIMUMPROPAGATOR_HPP_ */
