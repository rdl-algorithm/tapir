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

    virtual void propagate(HistorySequence *sequence, bool undo = false) = 0;
};

class AbstractBackpropagationStrategy : public BackpropagationStrategy {
public:
    AbstractBackpropagationStrategy(Solver *solver);
    virtual ~AbstractBackpropagationStrategy() = default;
    _NO_COPY_OR_MOVE(AbstractBackpropagationStrategy);

    virtual void updateEntry(HistoryEntry *entry, bool undo) = 0;
    virtual void updateEnd(HistoryEntry *entry, bool undo) = 0;
    virtual void updateRoot(HistoryEntry *entry, bool undo) = 0;

    virtual void propagate(HistorySequence *sequence, bool undo = false);
protected:
  Solver *solver_;
};

} /* namespace solver */

#endif /* SOLVER_BACKPROPAGATIONSTRATEGY_HPP_ */
