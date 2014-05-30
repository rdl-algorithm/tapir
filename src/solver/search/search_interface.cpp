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

/* ----------------------- StagedSearchStrategy ------------------------- */
StagedSearchStrategy::StagedSearchStrategy(Solver *solver,
        std::vector<std::unique_ptr<SearchStrategy>> strategies) :
            SearchStrategy(solver) {
}

std::unique_ptr<SearchInstance> StagedSearchStrategy::createSearchInstance(
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<StagedSearchInstance>();
}

/* ----------------------- StagedSearchInstance ------------------------- */
StagedSearchInstance::StagedSearchInstance(
        std::vector<std::unique_ptr<SearchStrategy>> const &strategies, HistorySequence *sequence,
        long maximumDepth, SearchStatus &status) :
            strategies_(strategies),
            sequence_(sequence),
            maximumDepth_(maximumDepth),
            status_(status) {
}

void StagedSearchInstance::extendSequence() {
    for (std::unique_ptr<SearchStrategy> const &strategy : strategies_) {
        std::unique_ptr<SearchInstance> searchInstance = strategy->createSearchInstance(sequence_,
                maximumDepth_, status_);
        searchInstance->extendSequence();
        if (status_ == SearchStatus::ERROR || status_ == SearchStatus::COMPLETE) {
            return;
        }
    }
}

/* ----------------------- ValueEstimator ------------------------- */
ValueEstimator::ValueEstimator(Solver *solver, std::function<double(HistoryEntry *)> estimator) :
            SearchStrategy(solver),
            estimator_(estimator) {
}

std::unique_ptr<SearchInstance> ValueEstimator::createSearchInstance(HistorySequence *sequence,
        long maximumDepth, SearchStatus &status) {
    return std::make_unique<ValueEstimatorInstance>(estimator_, sequence, status);
}

/* ----------------------- ValueEstimatorInstance ------------------------- */
ValueEstimatorInstance::ValueEstimatorInstance(std::function<double(HistoryEntry *)> estimator,
        HistorySequence *sequence, SearchStatus &status) :
            estimator_(estimator),
            sequence_(sequence),
            status_(status) {
}

void ValueEstimatorInstance::extendSequence() {
    if (status_ == SearchStatus::ROLLOUT_COMPLETE || status_ == SearchStatus::HIT_DEPTH_LIMIT) {
        HistoryEntry *lastEntry = sequence_->getLastEntry();
        double value = estimator_(lastEntry);
        lastEntry->immediateReward_ = value;
    }
    status_ = SearchStatus::COMPLETE;
}

/* ------------------- AbstractSearchInstance --------------------- */
AbstractSearchInstance::AbstractSearchInstance(Solver *solver, HistorySequence *sequence,
        long maximumDepth, SearchStatus &status) :
            status_(status),
            solver_(solver),
            model_(solver_->getModel()),
            sequence_(sequence),
            currentNode_(sequence->getLastEntry()->getAssociatedBeliefNode()),
            currentHistoricalData_(nullptr),
            discountFactor_(model_->getDiscountFactor()),
            maximumDepth_(maximumDepth) {
    status_ = SearchStatus::UNINITIALIZED;
}

void AbstractSearchInstance::initialize(BeliefNode */*currentNode*/) {
    status_ = SearchStatus::INITIAL;
}

void AbstractSearchInstance::extendSequence() {
    for (long currentDepth = currentNode_->getDepth();; currentDepth++) {
        if (currentDepth == maximumDepth_) {
            // We have hit the depth limit.
            status_ = SearchStatus::HIT_DEPTH_LIMIT;
            break;
        }
        std::unique_ptr<Action> action = getNextAction(currentEntry);
        if (action == nullptr) {
            break;
        }

        // If there was previously a reward for this entry, we must negate it.
        double oldReward = -currentEntry->immediateReward_;

        Model::StepResult result = model_->generateStep(*currentEntry->getState(), *action);
        currentEntry->immediateReward_ = result.reward;
        currentEntry->action_ = result.action->copy();
        currentEntry->transitionParameters_ = std::move(result.transitionParameters);
        currentEntry->observation_ = result.observation->copy();

        // Now we make a new history entry!
        // Add the next state to the pool
        StateInfo *nextStateInfo = solver_->getStatePool()->createOrGetInfo(*result.nextState);
        // Step forward in the history, and update the belief node.
        currentEntry = sequence_->addEntry(nextStateInfo);

        // Negate the old reward, and add a continuation.
        solver_->updateEstimate(currentNode_, -oldReward, +1);

        if (isFirst) {
            isFirst = false;
        } else {
            solver_->updateEstimate(currentNode_, 0, +1);
        }

        BeliefNode *nextNode = solver_->getPolicy()->createOrGetChild(currentNode_, *result.action,
                *result.observation);
        // Propagate the rewards in the tree.
        solver_->updateImmediate(currentNode_, *result.action, *result.observation, result.reward,
                +1);

        currentNode_ = nextNode;
        currentHistoricalData_ = nullptr;
        // Register the new history entry.
        currentEntry->registerNode(currentNode_);
        if (result.isTerminal) {
            status_ = SearchStatus::HIT_TERMINAL_STATE;
            break;
        }
    }
#pragma GCC warning "The heuristic estimate should be calculated here!"
    if (!isFirst) {
        solver_->updateEstimate(currentNode_, 0, 0);
    }
    status_ = finalize();
}

void AbstractSearchInstance::finalize() {
}

Solver *AbstractSearchInstance::getSolver() const {
    return solver_;
}

HistorySequence *AbstractSearchInstance::getSequence() const {
    return sequence_;
}

} /* namespace solver */
