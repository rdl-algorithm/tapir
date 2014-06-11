#include "DefaultHistoryCorrector.hpp"

#include <memory>

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"
#include "solver/StatePool.hpp"

#include "solver/abstract-problem/Model.hpp"
#include "solver/abstract-problem/State.hpp"

namespace solver {

DefaultHistoryCorrector::DefaultHistoryCorrector(Solver *solver, Heuristic heuristic) :
            HistoryCorrector(solver),
            heuristic_(heuristic) {
}

void DefaultHistoryCorrector::reviseSequence(HistorySequence *sequence) {
    if (sequence->endAffectedIdx_ < sequence->startAffectedIdx_) {
        debug::show_message("WARNING: Sequence to update has no affected entries!?");
        return;
    }

    // The ID of the entry at which this sequence goes down a different path in the tree to
    // its previous path.
    long divergingEntryId = -1;

    bool hitTerminalState = false; // True iff the sequence terminated prematurely.

    std::vector<std::unique_ptr<HistoryEntry>>::iterator historyIterator =
            (sequence->entrySequence_.begin() + sequence->startAffectedIdx_);
    std::vector<std::unique_ptr<HistoryEntry>>::iterator firstUnchanged =
            (sequence->entrySequence_.begin() + sequence->endAffectedIdx_ + 1);
    HistoryEntry *entry = nullptr;
    for (; historyIterator != firstUnchanged; historyIterator++) {
        entry = historyIterator->get();
        State const *state = entry->getState();

        // Check for early termination.
        hitTerminalState = getModel()->isTerminal(*state);
        if (hitTerminalState || entry->action_ == nullptr) {
            break;
        }

        // Deleted states should never occur in the update phase.
        if (changes::has_flag(entry->changeFlags_, ChangeFlags::DELETED)) {
            debug::show_message("ERROR: deleted state in updateSequence.");
        }

        HistoryEntry *nextEntry = (historyIterator + 1)->get();
        if (changes::has_flag(entry->changeFlags_, ChangeFlags::TRANSITION)) {
            entry->transitionParameters_ = getModel()->generateTransition(*entry->getState(),
                    *entry->action_);
            std::unique_ptr<State> nextState = getModel()->generateNextState(*entry->getState(),
                    *entry->action_, entry->transitionParameters_.get());
            if (!nextState->equals(*nextEntry->getState())) {
                StateInfo *nextStateInfo = getSolver()->getStatePool()->createOrGetInfo(*nextState);
                nextEntry->registerState(nextStateInfo);
                if (historyIterator + 1 == firstUnchanged) {
                    firstUnchanged++;
                }
                entry->setChangeFlags(ChangeFlags::OBSERVATION | ChangeFlags::REWARD);
                // Different state, so we must reset the flags.
                nextEntry->changeFlags_ = (ChangeFlags::REWARD | ChangeFlags::TRANSITION |
                        ChangeFlags::HEURISTIC);
                // Since it's a different state we may also need to update the heuristic.
            }
        }

        if (changes::has_flag(entry->changeFlags_, ChangeFlags::REWARD)) {
            double oldReward = entry->immediateReward_;
            entry->immediateReward_ = getModel()->generateReward(*entry->getState(),
                    *entry->action_, entry->transitionParameters_.get(), nextEntry->getState());

            // If we haven't diverged yet, we update the difference right now.
            if (divergingEntryId == -1 && entry->immediateReward_ != oldReward) {
                getSolver()->updateImmediate(entry->getAssociatedBeliefNode(),
                        *entry->action_, *entry->observation_,
                        entry->immediateReward_ - oldReward , 0);
            }

        }
        if (changes::has_flag(entry->changeFlags_, ChangeFlags::OBSERVATION)) {
            std::unique_ptr<Observation> newObservation = (getModel()->generateObservation(
                    entry->getState(), *entry->action_, entry->transitionParameters_.get(),
                    *nextEntry->getState()));
            if (!newObservation->equals(*entry->observation_)) {

                if (divergingEntryId == -1) {
                    divergingEntryId = entry->entryId_;
                    // We have diverged; this means the rest of the sequence should be negated.
                    getSolver()->updateSequence(sequence, -1, divergingEntryId, false);
                }

                entry->observation_ = std::move(newObservation);
            }
        }
        entry->resetChangeFlags(); // Reset the change flags for this entry.
    }

    if (entry->action_ == nullptr) {
        // We hit the last entry in the sequence => handle it as a special case.

        // We only have to deal with changes to the heuristic value if we've flagged it.
        if (changes::has_flag(entry->changeFlags_, ChangeFlags::HEURISTIC)) {
            double oldValue = entry->immediateReward_;
            if (hitTerminalState) {
                entry->immediateReward_ = 0;
            } else {
                entry->immediateReward_ = heuristic_(entry, entry->getState(),
                        entry->getAssociatedBeliefNode()->getHistoricalData());
            }

            // If no divergence occurred we update the difference in heuristic values here and now.
            if (divergingEntryId == -1) {
                getSolver()->updateEstimate(entry->getAssociatedBeliefNode(),
                        entry->immediateReward_ - oldValue, 0);
            }
        }
        entry->resetChangeFlags(); // Ensure the change flags for the last entry are reset.
    } else {
        // The changes did not reach the end of the sequence.

        if (hitTerminalState) { // Early termination!
            if (divergingEntryId == -1) {
                // If it has not yet been negated, negate the sequence from here onwards.
                getSolver()->updateSequence(sequence, -1, entry->entryId_, false);
            }

            // Now we have to erase all of the remaining entries in the sequence.
            sequence->erase(entry->entryId_ + 1);

            // Due to the early termination we must negate a continuation.
            getSolver()->updateEstimate(entry->getAssociatedBeliefNode(), 0, -1);

            // Now we must set the values for this entry properly.
            entry->action_ = nullptr;
            entry->transitionParameters_ = nullptr;
            entry->observation_ = nullptr;
            entry->immediateReward_ = 0;
        }

    }
    entry->resetChangeFlags(); // Reset the change flags for the last affected entry.

    // If there was divergence, we also have to update the rest of the sequence.
    if (divergingEntryId != -1) {
        historyIterator = sequence->entrySequence_.begin() + divergingEntryId;
        entry = historyIterator->get();
        BeliefNode *currentNode = entry->getAssociatedBeliefNode();
        // Update the node pointers for the rest of the sequence.
        while (entry->getAction() != nullptr) {
            currentNode = getSolver()->getPolicy()->createOrGetChild(currentNode,
                    *entry->getAction(), *entry->getObservation());
            historyIterator++;
            entry = historyIterator->get();
            entry->registerNode(currentNode);
        }

        // Now we backup the sequence.
        getSolver()->updateSequence(sequence, +1, divergingEntryId, false);
    }

    // Reset change flags for the sequence as a whole.
    sequence->resetChangeFlags();
}

} /* namespace solver */
