#include "search_interface.hpp"

#include <memory>

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"
#include "solver/StatePool.hpp"

#include "SearchStatus.hpp"

namespace solver {
/* ------------------- SearchInstance --------------------- */
SearchInstance::SearchInstance(SearchStatus &status) :
    status_(status) {
}

/* ------------------- StepGenerator --------------------- */
StepGenerator::StepGenerator(SearchStatus &status) :
    status_(status) {
}

/* ------------------- StagedSearchStrategy --------------------- */
StagedSearchStrategy::StagedSearchStrategy(Solver *solver,
        std::vector<std::unique_ptr<StepGeneratorFactory>> generatorFactories,
        std::function<double(HistoryEntry const *)> heuristic) :
            solver_(solver),
            generatorFactories_(generatorFactories),
            heuristic_(heuristic) {
}

std::unique_ptr<SearchInstance> StagedSearchStrategy::createSearchInstance(SearchStatus &status,
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<StagedSearchInstance>(status, sequence, maximumDepth, solver_,
            generatorFactories_, heuristic_);
}

/* ------------------- StagedSearchInstance --------------------- */
StagedSearchInstance::StagedSearchInstance(SearchStatus &status,
        HistorySequence *sequence, long maximumDepth, Solver *solver,
        std::vector<std::unique_ptr<StepGeneratorFactory>> const &generators,
        std::function<double(HistoryEntry const *)> heuristic) :
                SearchInstance(status),
            sequence_(sequence),
            maximumDepth_(maximumDepth),
            solver_(solver),
            model_(solver_->getModel()),
            generators_(generators),
            iterator_(generators.cbegin()),
            generator_(nullptr),
            heuristic_(heuristic) {
    generator_ = (*iterator_)->createGenerator(status_, sequence_);
    iterator_++;
}

void StagedSearchInstance::extendSequence() {
    if (status_ == SearchStatus::UNINITIALIZED) {
        debug::show_message("ERROR: Search failed to initialize!?");
        return;
    }

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

    std::vector<std::unique_ptr<StepGenerator>> generators;

    bool isFirst = true;
    while (true) {
        if (currentNode->getDepth() >= maximumDepth_) {
            status_ = SearchStatus::NEED_HEURISTIC;
            break;
        }

        // Step the search forward.
        Model::StepResult result = generator_->getStep();
        // Null action => continue to the next generator.
        if (result.action == nullptr) {
            // Not searching => done.
            if (status_ != SearchStatus::SEARCHING) {
                break;
            }
            // No more generators left => done.
            if (iterator_ == generators_.cend()) {
                status_ = SearchStatus::NEED_HEURISTIC;
                break;
            }
            // Get a new generator and keep searching.
            generator_ = (*iterator_)->createGenerator(status_, sequence_);
            iterator_++;
            continue;
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
