#include "SearchStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"

#include "SearchStatus.hpp"

namespace solver {

SearchInstance::SearchInstance(Solver *solver,
        BeliefNode *currentNode, HistorySequence *sequence,
        double discountFactor, long maximumDepth) :
                solver_(solver),
                model_(solver_->getModel()),
                currentNode_(currentNode),
                sequence_(sequence),
                discountFactor_(discountFactor),
                maximumDepth_(maximumDepth) {
}

SearchStatus SearchInstance::extendSequence() {
    HistoryEntry *currentEntry = sequence_->getEntry(
            sequence_->getLength() - 1);
    SearchStatus status = SearchStatus::INITIAL;
    bool done = model_->isTerminal(*currentEntry->getState());
    if (done) {
        debug::show_message("WARNING: Attempted to continue sequence from"
                " a terminal state.");
        sequence_->isTerminal_ = true;
        status = SearchStatus::TERMINATED;
    }
    long currentDepth = sequence_->startDepth_ + currentEntry->entryId_;
    double currentDiscount = currentEntry->discount_;
    for (; !done && currentDepth <= maximumDepth_; currentDepth++) {
        std::unique_ptr<Action> action;
        std::tie(status, action) = getStatusAndNextAction();
        if (action == nullptr) {
            done = true;
            break;
        }
        Model::StepResult result = model_->generateStep(
                *currentEntry->getState(), *action);
        if (result.isTerminal) {
            sequence_->isTerminal_ = true;
            done = true;
        }
        currentEntry->reward_ = result.reward;
        currentEntry->action_ = result.action->copy();
        currentEntry->transitionParameters_ = std::move(
                result.transitionParameters);
        currentEntry->observation_ = result.observation->copy();

        // Add the next state to the pool
        StateInfo *nextStateInfo = solver_->getStatePool()->createOrGetInfo(
                *result.nextState);
        // Step forward in the history, and update the belief node.
        currentDiscount *= discountFactor_;
        currentEntry = sequence_->addEntry(nextStateInfo, currentDiscount);
        currentNode_ = solver_->getPolicy()->createOrGetChild(
                currentNode_, *result.action, *result.observation);
        currentEntry->registerNode(currentNode_);
    }
    return status;
}

} /* namespace solver */
