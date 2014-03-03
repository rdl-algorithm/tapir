#include "DefaultHistoryCorrector.hpp"

#include <memory>

#include "HistoryEntry.hpp"
#include "HistorySequence.hpp"
#include "Model.hpp"
#include "Solver.hpp"
#include "StatePool.hpp"

#include "solver/geometry/State.hpp"

namespace solver {

DefaultHistoryCorrector::DefaultHistoryCorrector(Model *model) :
        HistoryCorrector(nullptr, model) {
}

void DefaultHistoryCorrector::reviseSequence(HistorySequence *sequence) {
    if (sequence->endAffectedIdx_ < sequence->startAffectedIdx_) {
        debug::show_message("WARNING: Sequence to update has no affected entries!?");
        return;
    }
    bool hitTerminalState = false;
    std::vector<std::unique_ptr<HistoryEntry>>::iterator historyIterator = (
        sequence->histSeq_.begin() + sequence->startAffectedIdx_);
    std::vector<std::unique_ptr<HistoryEntry>>::iterator firstUnchanged = (
            sequence->histSeq_.begin() + sequence->endAffectedIdx_ + 1);
    for (; historyIterator != firstUnchanged; historyIterator++) {
        HistoryEntry *entry = historyIterator->get();
        State const *state = entry->getState();

        hitTerminalState = model_->isTerminal(*state);
        if (hitTerminalState) {
            sequence->isTerminal_ = true;
        }
        if (hitTerminalState || entry->action_ == nullptr) {
            entry->action_ = nullptr;
            entry->observation_ = nullptr;
            entry->reward_ = 0;
            entry->totalDiscountedReward_ = 0;
            historyIterator++;
            break;
        }

        if (changes::hasFlag(entry->changeFlags_,
                        ChangeFlags::DELETED)) {
            debug::show_message("ERROR: deleted state in updateSequence.");
        }

        // Model::StepResult result;
        HistoryEntry *nextEntry = (historyIterator + 1)->get();
        if (changes::hasFlag(entry->changeFlags_, ChangeFlags::TRANSITION)) {
            entry->transitionParameters_ = model_->generateTransition(
                    *entry->getState(), *entry->action_);
            std::unique_ptr<State> nextState = model_->generateNextState(
                    *entry->getState(), *entry->action_,
                    entry->transitionParameters_.get());
            if (!nextState->equals(*nextEntry->getState())) {
                StateInfo *nextStateInfo = solver_->allStates_->createOrGetInfo(
                        *nextState);
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
        if (changes::hasFlag(entry->changeFlags_, ChangeFlags::REWARD)) {
            entry->reward_ = model_->generateReward(*entry->getState(),
                    *entry->action_, entry->transitionParameters_.get(),
                    nextEntry->getState());
        }
        if (changes::hasFlag(entry->changeFlags_, ChangeFlags::OBSERVATION )) {
            std::unique_ptr<Observation> newObservation = (
                    model_->generateObservation(entry->getState(),
                            *entry->action_,
                            entry->transitionParameters_.get(),
                            *nextEntry->getState()));
            if (!newObservation->equals(*entry->observation_)) {
                sequence->invalidLinksStartId_ = entry->entryId_;
                entry->observation_ = std::move(newObservation);
            }
        }
    }
    // If we hit a terminal state before the end of the sequence,
    // we must remove the remaining entries in the sequence.
    if (hitTerminalState && historyIterator != sequence->histSeq_.end()) {
        std::vector<std::unique_ptr<HistoryEntry>>::iterator firstDeletedEntry =
                historyIterator;
        for (; historyIterator != sequence->histSeq_.end(); historyIterator++) {
            (*historyIterator)->registerState(nullptr);
            (*historyIterator)->registerNode(nullptr);
        }
        sequence->histSeq_.erase(firstDeletedEntry, sequence->histSeq_.end());
    }
}

} /* namespace solver */
