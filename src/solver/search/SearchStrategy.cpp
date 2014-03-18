#include "SearchStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"
#include "solver/StatePool.hpp"

#include "SearchStatus.hpp"

namespace solver {
SearchStrategy::SearchStrategy(Solver *solver) :
    solver_(solver) {
}

AbstractSearchInstance::AbstractSearchInstance(Solver *solver,
        HistorySequence *sequence, long maximumDepth) :
                solver_(solver),
                model_(solver_->getModel()),
                sequence_(sequence),
                currentNode_(sequence->getLastEntry()->getAssociatedBeliefNode()),
                discountFactor_(model_->getDiscountFactor()),
                maximumDepth_(maximumDepth),
                status_(SearchStatus::UNINITIALIZED) {
}

SearchStatus AbstractSearchInstance::initialize() {
    status_ = SearchStatus::INITIAL;
    return status_;
}

SearchStatus AbstractSearchInstance::extendSequence() {
    if (status_ != SearchStatus::INITIAL) {
        debug::show_message("WARNING: Attempted to search without initializing.");
        return status_;
    }
    HistoryEntry *currentEntry = sequence_->getLastEntry();
    status_ = SearchStatus::INITIAL;
    if (model_->isTerminal(*currentEntry->getState())) {
        debug::show_message("WARNING: Attempted to continue sequence from"
                " a terminal state.");
        status_ = SearchStatus::HIT_TERMINAL_STATE;
        return status_;
    }
    for (long currentDepth = sequence_->getStartDepth() + currentEntry->entryId_;; currentDepth++) {
        if (currentDepth == maximumDepth_) {
            // We have hit the depth limit.
            status_ = SearchStatus::HIT_DEPTH_LIMIT;
            break;
        }
        std::unique_ptr<Action> action;
        std::tie(status_, action) = getStatusAndNextAction();
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

        // Now we make a new history entry!
        // Add the next state to the pool
        StateInfo *nextStateInfo = solver_->getStatePool()->createOrGetInfo(
                *result.nextState);
        // Step forward in the history, and update the belief node.
        currentEntry = sequence_->addEntry(nextStateInfo);
        currentNode_ = solver_->getPolicy()->createOrGetChild(
                currentNode_, *result.action, *result.observation);
        currentEntry->associatedBeliefNode_ = currentNode_;
        if (result.isTerminal) {
            status_ = SearchStatus::HIT_TERMINAL_STATE;
            break;
        }
    }
    return status_;
}

SearchStatus AbstractSearchInstance::finalize() {
    return status_;
}

} /* namespace solver */
