#include "SearchStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"

#include "SearchStatus.hpp"

namespace solver {
SearchInstance::SearchInstance(Solver *solver,  HistorySequence *sequence,
        BeliefNode *currentNode, double discountFactor, long maximumDepth) :
                solver_(solver),
                model_(solver_->getModel()),
                sequence_(sequence),
                currentNode_(currentNode),
                discountFactor_(discountFactor),
                maximumDepth_(maximumDepth) {
}

SearchInstance::SearchInstance(Solver *solver, HistorySequence *sequence) :
        SearchInstance(solver, sequence,
                sequence->getEntry(sequence->getLength()-1)->owningBeliefNode_,
                solver->getModel()->getDiscountFactor(),
                solver->getModel()->getMaximumDepth()) {
}

SearchStatus SearchInstance::extendSequence() {
    HistoryEntry *currentEntry = sequence_->getEntry(
            sequence_->getLength() - 1);
    SearchStatus status = SearchStatus::INITIAL;
    if (model_->isTerminal(*currentEntry->getState())) {
        debug::show_message("WARNING: Attempted to continue sequence from"
                " a terminal state.");
        sequence_->isTerminal_ = true;
        status = SearchStatus::HIT_TERMINAL_STATE;
        finishSearch(status);
        return status;
    }
    long currentDepth = sequence_->startDepth_ + currentEntry->entryId_;
    double currentDiscount = currentEntry->discount_;
    while (true) {
        if (currentDepth == maximumDepth_) {
            // We have hit the depth limit.
            status = SearchStatus::HIT_DEPTH_LIMIT;
            break;
        }
        std::unique_ptr<Action> action;
        std::tie(status, action) = getStatusAndNextAction();
        if (action == nullptr) {
            break;
        }
        Model::StepResult result = model_->generateStep(
                *currentEntry->getState(), *action);
        currentEntry->reward_ = result.reward;
        currentEntry->action_ = result.action->copy();
        currentEntry->transitionParameters_ = std::move(
                result.transitionParameters);
        currentEntry->observation_ = result.observation->copy();

        // Now we continue to the next history entry.
        currentDepth++;
        // Add the next state to the pool
        StateInfo *nextStateInfo = solver_->getStatePool()->createOrGetInfo(
                *result.nextState);
        // Step forward in the history, and update the belief node.
        currentDiscount *= discountFactor_;
        currentEntry = sequence_->addEntry(nextStateInfo, currentDiscount);
        currentNode_ = solver_->getPolicy()->createOrGetChild(
                currentNode_, *result.action, *result.observation);
        currentEntry->registerNode(currentNode_);
        if (result.isTerminal) {
            sequence_->isTerminal_ = true;
            status = SearchStatus::HIT_TERMINAL_STATE;
            break;
        }
    }
    finishSearch(status);
    return status;
}

void SearchInstance::finishSearch(SearchStatus status) {
}

} /* namespace solver */
