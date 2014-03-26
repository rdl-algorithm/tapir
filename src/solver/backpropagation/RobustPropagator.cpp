#include "RobustPropagator.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/ObservationMapping.hpp"

#include "BackpropagationStrategy.hpp"

namespace solver {
RobustPropagator::RobustPropagator(Solver *solver) :
        AbstractBackpropagationStrategy(solver) {
}

void RobustPropagator::updateEnd(HistoryEntry *entry, bool undo) {
    updateEntry(entry, undo);
}

void RobustPropagator::updateEntry(HistoryEntry *entry, bool undo) {
    BeliefNode *node = entry->getAssociatedBeliefNode();
    Action const &action = *entry->getAction();
    Observation const &observation = *entry->getObservation();

    ActionMapping *actionMapping = node->getMapping();
    ActionNode *actionNode = actionMapping->getActionNode(action);

    ObservationMapping *obsMapping = actionNode->getMapping();
    BeliefNode *childNode = obsMapping->getBelief(observation);

    long deltaNVisits;
    if (undo) {
        deltaNVisits = -1;
    } else {
        deltaNVisits = 1;
    }

    // Update the child's q-value calculation and the visit count.
    childNode->recalculateQValue();
    obsMapping->updateVisitCount(observation, deltaNVisits);

    // Now we update the action mapping.
    actionMapping->updateVisitCount(action, deltaNVisits);
    double deltaQ = entry->getCumulativeReward();
    if (undo) {
        deltaQ = -deltaQ;
    }
    actionMapping->updateTotalQValue(action, deltaQ);
}

void RobustPropagator::updateRoot(HistoryEntry *entry, bool /*undo*/) {
    entry->getAssociatedBeliefNode()->recalculateQValue();
}
} /* namespace solver */
