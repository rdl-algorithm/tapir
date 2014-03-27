#ifndef SOLVER_ROBUSTPROPAGATOR_HPP_
#define SOLVER_ROBUSTPROPAGATOR_HPP_

#include "BackpropagationStrategy.hpp"

namespace solver {

class RobustPropagator : public AbstractBackpropagationStrategy {
public:
    RobustPropagator(Solver *solver);
    virtual ~RobustPropagator() = default;
    virtual void updateEntry(HistoryEntry *entry, bool undo) override;
private:
    double deltaQ_;
    bool isSecondLastEntry_;
};

} /* namespace solver */

#endif /* SOLVER_ROBUSTPROPAGATOR_HPP_ */
