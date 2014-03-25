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

void AveragePropagator::updateEnd(HistoryEntry *entry, bool undo) {
    updateEntry(entry, undo);
}

void AveragePropagator::updateEntry(HistoryEntry *entry, bool undo) {
    long deltaNVisits;
    if (undo) {
        deltaNVisits = -1;
    } else {
        deltaNVisits = 1;
    }
    Action const &action = *entry->getAction();
    Observation const &observation = *entry->getObservation();
    BeliefNode *node = entry->getAssociatedBeliefNode();
    ActionMapping *actionMapping = node->getMapping();
    ActionNode *actionNode = actionMapping->getActionNode(action);
    ObservationMapping *obsMapping = actionNode->getMapping();
    BeliefNode *childNode = obsMapping->getBelief(observation);
    obsMapping->updateVisitCount(observation, deltaNVisits);

    childNode->recalculateQValue();

    actionMapping->updateVisitCount(action, deltaNVisits);
}
void AveragePropagator::updateRoot(HistoryEntry *entry, bool /*undo*/) {
    entry->getAssociatedBeliefNode()->recalculateQValue();
}
} /* namespace solver */
