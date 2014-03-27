#include "AveragePropagator.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/ObservationMapping.hpp"

#include "BackpropagationStrategy.hpp"

namespace solver {
AveragePropagator::AveragePropagator(Solver *solver) :
        AbstractBackpropagationStrategy(solver) {
}

void AveragePropagator::updateEntry(HistoryEntry *entry, bool undo) {
    // If the entry is the last in the sequence, we don't update.
    if (entry->getAction() == nullptr) {
        return;
    }

    // Derived values
    Action const &action = *entry->getAction();
    ActionMapping *actionMapping = entry->getAssociatedBeliefNode()->getMapping();
    long deltaNVisits = undo ? -1 : 1;

    // Update the visit counts for the current node.
    actionMapping->updateVisitCount(action, deltaNVisits);
    actionMapping->getActionNode(action)->getMapping()->updateVisitCount(
            *entry->getObservation(), deltaNVisits);

    // Calculate the q-value based on the cumulative reward for this sequence,
    // from this point onward.
    double deltaQ = entry->getCumulativeReward();
    if (undo) {
        deltaQ = -deltaQ;
    }

    // Update the mapping and force it to recalculate.
    actionMapping->updateTotalQValue(action, deltaQ);
    actionMapping->update();
}
} /* namespace solver */
