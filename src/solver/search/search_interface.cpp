#include "solver/search/search_interface.hpp"

#include <functional>
#include <memory>

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"
#include "solver/StatePool.hpp"

#include "solver/search/SearchStatus.hpp"

#include "solver/search/heuristics/heuristics.hpp"

namespace solver {
/* ------------------- SearchInstance --------------------- */
SearchInstance::SearchInstance(SearchStatus &status) :
            status_(status) {
}

/* ------------------- StepGenerator --------------------- */
StepGenerator::StepGenerator(SearchStatus &status) :
            status_(status) {
    status_ = SearchStatus::INITIAL;
}

/* ------------------- StagedStepGenerator --------------------- */
StagedStepGenerator::StagedStepGenerator(SearchStatus &status, HistorySequence *sequence,
        std::vector<std::unique_ptr<StepGeneratorFactory>> const &generatorSequence) :
            StepGenerator(status),
            sequence_(sequence),
            generatorSequence_(generatorSequence),
            iterator_(generatorSequence_.cbegin()),
            generator_(nullptr) {
    generator_ = (*iterator_)->createGenerator(status_, sequence_);
    iterator_++;
}

Model::StepResult StagedStepGenerator::getStep() {
    Model::StepResult result = generator_->getStep();
    while (status_ == SearchStatus::STAGE_COMPLETE && result.action == nullptr) {
        if (iterator_ == generatorSequence_.cend()) {
            break;
        }
        generator_ = (*iterator_)->createGenerator(status_, sequence_);
        iterator_++;
        result = generator_->getStep();
    }
    return result;
}

/* ------------------- StagedStepGeneratorFactory --------------------- */
StagedStepGeneratorFactory::StagedStepGeneratorFactory(
        std::vector<std::unique_ptr<StepGeneratorFactory>> generatorSequence) :
            generatorSequence_(std::move(generatorSequence)) {
}

std::unique_ptr<StepGenerator> StagedStepGeneratorFactory::createGenerator(SearchStatus &status,
        HistorySequence *sequence) {
    return std::make_unique<StagedStepGenerator>(status, sequence, generatorSequence_);
}

/* ------------------- AbstractSearchStrategy --------------------- */
BasicSearchStrategy::BasicSearchStrategy(Solver *solver,
        std::unique_ptr<StepGeneratorFactory> generatorFactory,
        std::unique_ptr<Heuristic> heuristic) :
            solver_(solver),
            generatorFactory_(std::move(generatorFactory)),
            heuristic_(std::move(heuristic)) {
}

std::unique_ptr<SearchInstance> BasicSearchStrategy::createSearchInstance(SearchStatus &status,
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<BasicSearchInstance>(status, sequence, maximumDepth, solver_,
            generatorFactory_->createGenerator(status, sequence), heuristic_.get());
}

/* ------------------- AbstractSearchInstance --------------------- */
BasicSearchInstance::BasicSearchInstance(SearchStatus &status, HistorySequence *sequence,
        long maximumDepth, Solver *solver, std::unique_ptr<StepGenerator> generator,
        Heuristic *heuristic) :
            SearchInstance(status),
            sequence_(sequence),
            maximumDepth_(maximumDepth),
            solver_(solver),
            model_(solver_->getModel()),
            generator_(std::move(generator)),
            heuristic_(heuristic) {
}

void BasicSearchInstance::extendSequence() {
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
    bool isFirst = true;
    while (true) {
        if (currentNode->getDepth() >= maximumDepth_) {
            status_ = SearchStatus::STAGE_COMPLETE;
            break;
        }

        // Step the search forward.
        Model::StepResult result = generator_->getStep();
        // Null action => stop the search.
        if (result.action == nullptr) {
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

        // Create the child belief node and do an update.
        BeliefNode *nextNode = solver_->getPolicy()->createOrGetChild(currentNode, *result.action,
                        *result.observation);
        solver_->updateImmediate(currentNode, *result.action, *result.observation, result.reward,
                +1);
        currentNode = nextNode;

        // Now we create a new history entry and step the history forward.
        StateInfo *nextStateInfo = solver_->getStatePool()->createOrGetInfo(*result.nextState);
        currentEntry = sequence_->addEntry(nextStateInfo);
        currentEntry->registerNode(currentNode);

        if (result.isTerminal) {
            status_ = SearchStatus::FINISHED;
            break;
        }
    }

    // If we require a heuristic estimate, calculate it.
    if (status_ == SearchStatus::STAGE_COMPLETE) {
        currentEntry->immediateReward_ = heuristic_->getHeuristicValue(currentEntry);
        // Use the heuristic value to update the estimate of the parent belief node.
        solver_->updateEstimate(currentNode, currentEntry->immediateReward_, 0);
        status_ = SearchStatus::FINISHED;
    } else if (status_ == SearchStatus::FINISHED) {
        // Finished normally; no problems.
    } else if (status_ == SearchStatus::UNINITIALIZED) {
        debug::show_message("ERROR: Search algorithm could not initialize.");
    } else if (status_ == SearchStatus::INITIAL) {
        debug::show_message("ERROR: Search algorithm initialized but did not run.");
    } else if (status_ == SearchStatus::ERROR) {
        debug::show_message("ERROR: Error in search algorithm!");
    } else {
        debug::show_message("ERROR: Invalid search status.");
    }
}
} /* namespace solver */
