#ifndef SOLVER_BACKPROPAGATIONSTRATEGY_HPP_
#define SOLVER_BACKPROPAGATIONSTRATEGY_HPP_

#include "global.hpp"

namespace solver {
class ActionNode;
class BeliefNode;
class HistoryEntry;
class HistorySequence;
class Solver;

class BackpropagationStrategy {
  public:
    BackpropagationStrategy() = default;
    virtual ~BackpropagationStrategy() = default;

    /** Backpropagates along the given sequence, from the end of the sequence.
     * the "undo" flag is used to represent a negative backup
     * (i.e. sequence removal).
     */
    virtual void propagate(HistorySequence *sequence, bool undo = false) = 0;
};

class AbstractBackpropagationStrategy : public BackpropagationStrategy {
public:
    AbstractBackpropagationStrategy(Solver *solver);
    virtual ~AbstractBackpropagationStrategy() = default;
    _NO_COPY_OR_MOVE(AbstractBackpropagationStrategy);

    /** Updates w.r.t. an individual history entry. */
    virtual void updateEntry(HistoryEntry *entry, bool undo) = 0;

    virtual void propagate(HistorySequence *sequence, bool undo = false);

    /** Returns the solver associated with this backprop. strategy. */
    virtual Solver *getSolver() const;
private:
  Solver *solver_;
};

} /* namespace solver */

#endif /* SOLVER_BACKPROPAGATIONSTRATEGY_HPP_ */
