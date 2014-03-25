#include "BackpropagationStrategy.hpp"

#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

namespace solver {
AbstractBackpropagationStrategy::AbstractBackpropagationStrategy(Solver *solver) :
        solver_(solver) {
}

void AbstractBackpropagationStrategy::propagate(HistorySequence *sequence, bool undo) {
    std::vector<std::unique_ptr<HistoryEntry>>::reverse_iterator itHist = (
                sequence->histSeq_.rbegin());
    (*itHist)->hasBeenBackedUp_ = !undo;
    itHist++;
    if (itHist != sequence->histSeq_.rend()) {
        // Update the last action.
        updateEnd(itHist->get(), undo);
        (*itHist)->hasBeenBackedUp_ = !undo;
        itHist++;
    }
    for (; itHist != sequence->histSeq_.rend(); itHist++) {
        // Propagate back along the sequence.
        HistoryEntry *entry = itHist->get();
        updateEntry(entry, undo);
        (*itHist)->hasBeenBackedUp_ = !undo;
    }
    // Update the root.
    updateRoot(sequence->getFirstEntry(), undo);
}

} /* namespace solver */
