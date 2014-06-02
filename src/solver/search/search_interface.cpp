#include "search_interface.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"
#include "solver/StatePool.hpp"

#include "SearchStatus.hpp"

namespace solver {
/* ----------------------- SearchStrategy ------------------------- */
SearchStrategy::SearchStrategy(Solver *solver) :
            solver_(solver) {
}

Solver *SearchStrategy::getSolver() const {
    return solver_;
}

/* ----------------------- SearchInstance ------------------------- */
SearchInstance::SearchInstance(SearchStatus &status) :
            status_(status) {
}

/* ------------------- BasicSearchInstance --------------------- */
BasicSearchInstance::BasicSearchInstance(SearchStatus &status, HistorySequence *sequence,
        long maximumDepth, Solver *solver, std::unique_ptr<StepGenerator> stepGenerator,
        std::function<double(HistoryEntry const *)> heuristic) :
            SearchInstance(status),
            sequence_(sequence),
            maximumDepth_(maximumDepth),
            solver_(solver),
            model_(solver_->getModel()),
            stepGenerator_(stepGenerator),
            heuristic_(heuristic) {
    status_ = SearchStatus::INITIAL;
}

void BasicSearchInstance::extendSequence() {
    HistoryEntry *currentEntry = sequence_->getLastEntry();
    if (model_->isTerminal(*currentEntry->getState())) {
        debug::show_message("WARNING: Attempted to continue sequence"
                " from a terminal state.");
        return;
    } else if (currentEntry->getAction() != nullptr) {
        debug::show_message("ERROR: The last in the sequence already has an action!?");
        return;
    }

    BeliefNode *currentNode = currentEntry->getAssociatedBeliefNode();

    bool isFirst = true;
    status_ = SearchStatus::SEARCHING;
    while (true) {
        if (currentNode->getDepth() >= maximumDepth_) {
            status_ = SearchStatus::NEED_HEURISTIC;
            break;
        }

        // Step the search forward.
        Model::StepResult result = stepGenerator_->getStep();

        if (result.action == nullptr) {
            status_ = SearchStatus::NEED_HEURISTIC;
            break;
        }

        if (isFirst) {
            isFirst = false;
        } else {
            // We're continuing, so we must add a continuation.
            solver_->updateEstimate(currentNode, 0, +1);
        }
        currentEntry->immediateReward_ = result.reward;
        currentEntry->action_ = result.action->copy();
        currentEntry->transitionParameters_ = std::move(result.transitionParameters);
        currentEntry->observation_ = result.observation->copy();

        // Update with the immediate reward.
        solver_->updateImmediate(currentNode, *result.action, *result.observation, result.reward,
                +1);

        // Now we make a new history entry!
        // Add the next state to the pool
        StateInfo *nextStateInfo = solver_->getStatePool()->createOrGetInfo(*result.nextState);
        // Step forward in the history, and update the belief node.
        currentEntry = sequence_->addEntry(nextStateInfo);
        BeliefNode *nextNode = solver_->getPolicy()->createOrGetChild(currentNode, *result.action,
                *result.observation);
        currentNode = nextNode;

        if (result.isTerminal) {
            status_ = SearchStatus::FINISHED;
            break;
        }
    }

    // If we require a heuristic estimate, calculate it.
    if (status_ == SearchStatus::NEED_HEURISTIC) {
        currentEntry->immediateReward_ = heuristic_(currentEntry);
        // Use the heuristic value to update the estimate of the parent belief node.
        solver_->updateEstimate(currentNode, currentEntry->immediateReward_, 0);
        status_ = SearchStatus::FINISHED;
    } else if (status_ == SearchStatus::UNINITIALIZED) {
        debug::show_message("ERROR: Search algorithm could not initialize!?");
    } else if (status_ == SearchStatus::ERROR) {
        debug::show_message("ERROR: Error in search algorithm!");
    } else if (status_ != SearchStatus::FINISHED) {
        debug::show_message("ERROR: Search failed to complete!");
    }
}
} /* namespace solver */
