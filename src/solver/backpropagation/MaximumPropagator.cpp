#include "MaximumPropagator.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/ObservationMapping.hpp"

#include "BackpropagationStrategy.hpp"

namespace solver {
MaximumPropagator::MaximumPropagator(Solver *solver) :
        AbstractBackpropagationStrategy(solver) {
}

void MaximumPropagator::updateEnd(HistoryEntry *entry, bool undo) {
    BeliefNode *node = entry->getAssociatedBeliefNode();
    Action const &action = *entry->getAction();
    Observation const &observation = *entry->getObservation();

    ActionMapping *actionMapping = node->getMapping();
    ActionNode *actionNode = actionMapping->getActionNode(action);
    ObservationMapping *obsMapping = actionNode->getMapping();

    long deltaNVisits;
    if (undo) {
        deltaNVisits = -1;
    } else {
        deltaNVisits = 1;
    }

    // Update the visit count of the observation.
    obsMapping->updateVisitCount(observation, deltaNVisits);

    // Now we update the action mapping.
    actionMapping->updateVisitCount(action, deltaNVisits);
    double deltaQ = entry->getCumulativeReward();
    if (undo) {
        deltaQ = -deltaQ;
    }
    actionMapping->updateTotalQValue(action, deltaQ);
}

void MaximumPropagator::updateEntry(HistoryEntry *entry, bool undo) {
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

    // Retrieve the visit counts and discount factor.
    long newVisitCount = childNode->getMapping()->getTotalVisitCount();
    long oldVisitCount = newVisitCount - deltaNVisits;
    double discountFactor = solver_->getModel()->getDiscountFactor();

    // Calculated change in the total q-value of the action.
    long deltaQ = 0;

    // Calculate the previous contribution to the q-value.
    double oldChildQ = childNode->getQValue();
    if (oldVisitCount > 0) {
        // Remove the value previously associated with the child's Q-value.
        deltaQ = -discountFactor * oldVisitCount * oldChildQ;
        if (!std::isfinite(oldChildQ)) {
            debug::show_message("ERROR: Non-finite old Q!");
        }
    }

    // Update the child's q-value calculation and the observation visit count.
    childNode->recalculateQValue();
    obsMapping->updateVisitCount(observation, deltaNVisits);

    // Calculate the new contribution to the q-value.
    double newChildQ = childNode->getQValue();
    if (newVisitCount > 0) {
        deltaQ += discountFactor * newVisitCount * newChildQ;
        if (!std::isfinite(newChildQ)) {
            debug::show_message("ERROR: Non-finite new Q!");
        }
    }

    // Now add in the contribution from the immediate reward
    if (undo) {
        deltaQ -= entry->getReward();
    } else {
        deltaQ += entry->getReward();
    }

    // Now update the viist count and q-value for the action.
    actionMapping->updateVisitCount(action, deltaNVisits);
    actionMapping->updateTotalQValue(action, deltaQ);
}
void MaximumPropagator::updateRoot(HistoryEntry *entry, bool /*undo*/) {
    // Update the q-value of the root.
    entry->getAssociatedBeliefNode()->recalculateQValue();
}
} /* namespace solver */
