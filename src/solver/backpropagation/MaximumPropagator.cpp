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
        AbstractBackpropagationStrategy(solver),
        deltaQ_(0),
        isSecondLastEntry_(false) {
}

void MaximumPropagator::updateEntry(HistoryEntry *entry, bool undo) {
    // If the entry is the last in the sequence, we don't update the node.
    if (entry->getAction() == nullptr) {
        deltaQ_ = 0;
        isSecondLastEntry_ = true;
        return;
    }

    // Derived values.
    BeliefNode *currentNode = entry->getAssociatedBeliefNode();
    Action const &action = *entry->getAction();
    ActionMapping *actionMap = currentNode->getMapping();
    ObservationMapping *obsMap = actionMap->getActionNode(action)->getMapping();
    long deltaNVisits = undo ? -1 : 1;
    double discountFactor = solver_->getModel()->getDiscountFactor();

    // We base our visit counts off the number of visits from this node
    // that did not begin in this node.
    long oldVisitCount = actionMap->getTotalVisitCount();
    oldVisitCount -= currentNode->getNumberOfStartingSequences();
    long newVisitCount = oldVisitCount + deltaNVisits;

    // Update the visit counts for the current node.
    actionMap->updateVisitCount(action, deltaNVisits);
    obsMap->updateVisitCount(*entry->getObservation(), deltaNVisits);

    // This will hold the effect of the changes to the current node on its parent.
    double parentDeltaQ = 0;

    // Remove the old contribution, if any
    if (oldVisitCount > 0) {
        double q = actionMap->getMaxQValue();
        if (std::isfinite(q)) {
            parentDeltaQ -= discountFactor * oldVisitCount * q;
        } else {
            debug::show_message("ERROR: non-finite old Q!");
        }
    }

    // Include the immediate reward
    double immediateQ;
    if (isSecondLastEntry_) {
        // The second-last entry should also include the heuristic value
        immediateQ = entry->getCumulativeReward();
    } else {
        immediateQ = entry->getReward();
    }
    if (undo) {
        deltaQ_ -= immediateQ;
    } else {
        deltaQ_ += immediateQ;
    }

    // Update the mapping and force it to recalculate.
    actionMap->updateTotalQValue(action, deltaQ_);
    actionMap->update();

    // Add the new contribution, if any.
    if (newVisitCount > 0) {
        double q = actionMap->getMaxQValue();
        if (std::isfinite(q)) {
            parentDeltaQ += discountFactor * newVisitCount * q;
        } else {
            debug::show_message("ERROR: non-finite new Q!");
        }
    }

    // Store the calculated value.
    deltaQ_ = parentDeltaQ;

    // If we were, we're not in the second-last entry any more.
    isSecondLastEntry_ = false;
}
} /* namespace solver */
