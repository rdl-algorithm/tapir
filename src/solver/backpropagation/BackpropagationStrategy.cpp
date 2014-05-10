#include "BackpropagationStrategy.hpp"

#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

namespace solver {
AbstractBackpropagationStrategy::AbstractBackpropagationStrategy(Solver *solver) :
        solver_(solver) {
}

void AbstractBackpropagationStrategy::propagate(HistorySequence *sequence, bool undo) {
    for (std::vector<std::unique_ptr<HistoryEntry>>::reverse_iterator itHist = sequence->entrySequence_.rbegin();
            itHist != sequence->entrySequence_.rend(); itHist++) {
        HistoryEntry *entry = itHist->get();
        updateEntry(entry, undo);
        entry->hasBeenBackedUp_ = !undo;
    }
}

Solver *AbstractBackpropagationStrategy::getSolver() const {
    return solver_;
}
} /* namespace solver */
