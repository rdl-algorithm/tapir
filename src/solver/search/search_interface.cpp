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

/* ------------------- AbstractSearchInstance --------------------- */
AbstractSearchInstance::AbstractSearchInstance(Solver *solver,
        HistorySequence *sequence, long maximumDepth) :
                solver_(solver),
                model_(solver_->getModel()),
                sequence_(sequence),
                currentNode_(sequence->getLastEntry()->getAssociatedBeliefNode()),
                currentHistoricalData_(nullptr),
                discountFactor_(model_->getDiscountFactor()),
                maximumDepth_(maximumDepth),
                status_(SearchStatus::UNINITIALIZED) {
}

SearchStatus AbstractSearchInstance::initialize(BeliefNode */*currentNode*/) {
    status_ = SearchStatus::INITIAL;
    return status_;
}

SearchStatus AbstractSearchInstance::getStatus() const {
    return status_;
}

void AbstractSearchInstance::extendSequence() {
    status_ = initialize(currentNode_);
    if (status_ != SearchStatus::INITIAL) {
        debug::show_message("WARNING: Attempted to search without initializing.");
        return;
    }
    HistoryEntry *currentEntry = sequence_->getLastEntry();
    status_ = SearchStatus::INITIAL;
    if (model_->isTerminal(*currentEntry->getState())) {
        debug::show_message("WARNING: Attempted to continue sequence from"
                " a terminal state.");
        status_ = SearchStatus::HIT_TERMINAL_STATE;
        return;
    }

    for (long currentDepth = currentNode_->getDepth();; currentDepth++) {
        if (currentDepth == maximumDepth_) {
            // We have hit the depth limit.
            status_ = SearchStatus::HIT_DEPTH_LIMIT;
            break;
        }
        SearchStep step = getSearchStep();
        status_ = step.status;
        if (step.action == nullptr) {
            break;
        }

        Model::StepResult result = model_->generateStep(
                *currentEntry->getState(), *step.action);
        currentEntry->immediateReward_ = result.reward;
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

        if (currentNode_ != nullptr) {
            solver_->updateEstimate(currentNode_, 0, +1);
            if (step.createNode) {
                BeliefNode *newNode = solver_->getPolicy()->createOrGetChild(
                        currentNode_, *result.action, *result.observation);

                // Propagate the rewards in the tree.
                solver_->updateImmediate(currentNode_, *result.action, *result.observation,
                        result.reward, +1);

                currentNode_ = newNode;
                currentHistoricalData_ = nullptr;
                // Register the new history entry.
                currentEntry->registerNode(currentNode_);
            }
        } else {
            HistoricalData *oldData;
            if (currentNode_ == nullptr) {
                oldData = currentHistoricalData_.get();
            } else {
                oldData = currentNode_->getHistoricalData();
                currentNode_ = nullptr;
            }
            if (oldData != nullptr) {
                currentHistoricalData_ = oldData->createChild(
                        *result.action, *result.observation);
            } else {
                currentHistoricalData_ = nullptr;
            }
        }
        if (result.isTerminal) {
            status_ = SearchStatus::HIT_TERMINAL_STATE;
            break;
        }
    }
#pragma GCC warning "The heuristic estimate should be calculated here!"
    solver_->updateEstimate(currentNode_, 0, 0);
    status_ = finalize();
}

SearchStatus AbstractSearchInstance::finalize() {
    return status_;
}

Solver *AbstractSearchInstance::getSolver() const {
    return solver_;
}

HistorySequence *AbstractSearchInstance::getSequence() const {
    return sequence_;
}

/* ------------------- AbstractSelectionInstance --------------------- */
AbstractSelectionInstance::AbstractSelectionInstance(Solver *solver,
        HistorySequence *sequence, long maximumDepth) :
        AbstractSearchInstance(solver, sequence, maximumDepth) {
}

SearchStep AbstractSelectionInstance::getSearchStep() {
    return getSearchStep(currentNode_);
}

/* ------------------- AbstractRolloutInstance --------------------- */
AbstractRolloutInstance::AbstractRolloutInstance(Solver *solver,
        HistorySequence *sequence, long maximumDepth) :
        AbstractSearchInstance(solver, sequence, maximumDepth) {
}

SearchStep AbstractRolloutInstance::getSearchStep() {
    HistoricalData *currentData;
    if (currentNode_ != nullptr) {
        currentData = currentNode_->getHistoricalData();
    } else {
        currentData = currentHistoricalData_.get();
    }
    return getSearchStep(currentData);
}

} /* namespace solver */
