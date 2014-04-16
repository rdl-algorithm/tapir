#include "DefaultHistoryCorrector.hpp"

#include <memory>

#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"
#include "solver/StatePool.hpp"

#include "solver/abstract-problem/Model.hpp"
#include "solver/abstract-problem/State.hpp"

namespace solver {

DefaultHistoryCorrector::DefaultHistoryCorrector(Solver *solver) :
        HistoryCorrector(solver) {
}

void DefaultHistoryCorrector::reviseSequence(HistorySequence *sequence) {
    if (sequence->endAffectedIdx_ < sequence->startAffectedIdx_) {
        debug::show_message("WARNING: Sequence to update has no affected entries!?");
        return;
    }
    bool hitTerminalState = false;
    std::vector<std::unique_ptr<HistoryEntry>>::iterator historyIterator = (
        sequence->entrySequence_.begin() + sequence->startAffectedIdx_);
    std::vector<std::unique_ptr<HistoryEntry>>::iterator firstUnchanged = (
            sequence->entrySequence_.begin() + sequence->endAffectedIdx_ + 1);
    for (; historyIterator != firstUnchanged; historyIterator++) {
        HistoryEntry *entry = historyIterator->get();
        State const *state = entry->getState();

        hitTerminalState = getModel()->isTerminal(*state);
        if (hitTerminalState || entry->action_ == nullptr) {
            entry->action_ = nullptr;
            entry->observation_ = nullptr;
            entry->reward_ = 0;
            entry->rewardFromHere_ = 0;
            historyIterator++;
            break;
        }

        if (changes::has_flag(entry->changeFlags_,
                        ChangeFlags::DELETED)) {
            debug::show_message("ERROR: deleted state in updateSequence.");
        }

        // Model::StepResult result;
        HistoryEntry *nextEntry = (historyIterator + 1)->get();
        if (changes::has_flag(entry->changeFlags_, ChangeFlags::TRANSITION)) {
            entry->transitionParameters_ = getModel()->generateTransition(
                    *entry->getState(), *entry->action_);
            std::unique_ptr<State> nextState = getModel()->generateNextState(
                    *entry->getState(), *entry->action_,
                    entry->transitionParameters_.get());
            if (!nextState->equals(*nextEntry->getState())) {
                StateInfo *nextStateInfo = getSolver()->getStatePool()->createOrGetInfo(*nextState);
                nextEntry->registerState(nextStateInfo);
                if (historyIterator + 1 == firstUnchanged) {
                    firstUnchanged++;
                }
                entry->setChangeFlags(
                        ChangeFlags::OBSERVATION | ChangeFlags::REWARD);
                nextEntry->changeFlags_ = (ChangeFlags::REWARD
                        | ChangeFlags::TRANSITION);
            }
        }
        if (changes::has_flag(entry->changeFlags_, ChangeFlags::REWARD)) {
            entry->reward_ = getModel()->generateReward(*entry->getState(),
                    *entry->action_, entry->transitionParameters_.get(),
                    nextEntry->getState());
        }
        if (changes::has_flag(entry->changeFlags_, ChangeFlags::OBSERVATION )) {
            std::unique_ptr<Observation> newObservation = (
                    getModel()->generateObservation(entry->getState(),
                            *entry->action_,
                            entry->transitionParameters_.get(),
                            *nextEntry->getState()));
            if (!newObservation->equals(*entry->observation_)) {
                entry->observation_ = std::move(newObservation);
            }
        }
    }
    // If we hit a terminal state before the end of the sequence,
    // we must remove the remaining entries in the sequence.
    if (hitTerminalState && historyIterator != sequence->entrySequence_.end()) {
        std::vector<std::unique_ptr<HistoryEntry>>::iterator firstDeletedEntry =
                historyIterator;
        for (; historyIterator != sequence->entrySequence_.end(); historyIterator++) {
            (*historyIterator)->registerState(nullptr);
        }
        sequence->entrySequence_.erase(firstDeletedEntry, sequence->entrySequence_.end());
    }
}

} /* namespace solver */
