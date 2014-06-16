#include "solver/search/search_interface.hpp"

#include <functional>
#include <memory>

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"
#include "solver/StatePool.hpp"

#include "solver/abstract-problem/heuristics/Heuristic.hpp"

#include "solver/search/SearchStatus.hpp"

namespace solver {
/* ----------------------- StepGenerator ------------------------- */
StepGenerator::StepGenerator(SearchStatus &status) :
            status_(status) {
}

/* ------------------- StagedStepGeneratorFactory --------------------- */
StagedStepGeneratorFactory::StagedStepGeneratorFactory(
        std::vector<std::unique_ptr<StepGeneratorFactory>> factories) :
            factories_(std::move(factories)) {
}

std::unique_ptr<StepGenerator> StagedStepGeneratorFactory::createGenerator(SearchStatus &status,
        HistoryEntry const *entry, State const *state, HistoricalData const *data) {
    return std::make_unique<StagedStepGenerator>(status, factories_, entry, state, data);
}

/* ------------------- StagedStepGenerator --------------------- */
StagedStepGenerator::StagedStepGenerator(SearchStatus &status,
        std::vector<std::unique_ptr<StepGeneratorFactory>> const &factories,
        HistoryEntry const *entry, State const *state, HistoricalData const *data) :
            StepGenerator(status),
            factories_(factories),
            iterator_(factories.cbegin()),
            generator_(nullptr) {
    generator_ = (*iterator_)->createGenerator(status_, entry, state, data);
    iterator_++;
}

Model::StepResult StagedStepGenerator::getStep(HistoryEntry const *entry, State const *state,
        HistoricalData const *data) {
    Model::StepResult result = generator_->getStep(entry, state, data);
    while (result.action == nullptr) {
        if (status_ == SearchStatus::FINISHED || iterator_ == factories_.cend()) {
            generator_ = nullptr;
            return result;
        }

        generator_ = (*iterator_)->createGenerator(status_, entry, state, data);
        iterator_++;
        result = generator_->getStep(entry, state, data);
    }
    return result;
}

/* ------------------- AbstractSearchStrategy --------------------- */
BasicSearchStrategy::BasicSearchStrategy(Solver *solver,
        std::unique_ptr<StepGeneratorFactory> factory, Heuristic heuristic) :
            solver_(solver),
            factory_(std::move(factory)),
            heuristic_(heuristic) {
}

SearchStatus BasicSearchStrategy::extendSequence(HistorySequence *sequence, long maximumDepth,
        bool doBackup) {
    HistoryEntry *currentEntry = sequence->getLastEntry();
    BeliefNode *currentNode = currentEntry->getAssociatedBeliefNode();

    SearchStatus status = SearchStatus::UNINITIALIZED;
    std::unique_ptr<StepGenerator> generator = factory_->createGenerator(status, currentEntry,
            currentEntry->getState(), currentNode->getHistoricalData());
    if (status == SearchStatus::UNINITIALIZED) {
        // Failure to initialize => return this status.
        return status;
    }

    Model *model = solver_->getModel();
    if (model->isTerminal(*currentEntry->getState())) {
        debug::show_message("WARNING: Attempted to continue sequence"
                " from a terminal state.");
        return SearchStatus::ERROR;
    } else if (currentEntry->getAction() != nullptr) {
        debug::show_message("ERROR: The last in the sequence already has an action!?");
        return SearchStatus::ERROR;
    }

    while (true) {
        if (currentNode->getDepth() >= maximumDepth) {
            status = SearchStatus::OUT_OF_STEPS;
            break;
        }
        // Step the search forward.
        Model::StepResult result = generator->getStep(currentEntry, currentEntry->getState(),
                currentNode->getHistoricalData());

        // Null action => stop the search.
        if (result.action == nullptr) {
            break;
        }

        currentEntry->immediateReward_ = result.reward;
        currentEntry->action_ = result.action->copy();
        currentEntry->transitionParameters_ = std::move(result.transitionParameters);
        currentEntry->observation_ = result.observation->copy();

        // Create the child belief node and do an update.
        BeliefNode *nextNode = solver_->getPolicy()->createOrGetChild(currentNode, *result.action,
                *result.observation);
        currentNode = nextNode;

        // Now we create a new history entry and step the history forward.
        StateInfo *nextStateInfo = solver_->getStatePool()->createOrGetInfo(*result.nextState);
        currentEntry = sequence->addEntry();
        currentEntry->registerState(nextStateInfo);
        currentEntry->registerNode(currentNode);

        if (result.isTerminal) {
            status = SearchStatus::FINISHED;
            break;
        }
    }


    if (status == SearchStatus::OUT_OF_STEPS) {
        // If we require a heuristic estimate, calculate it.
        currentEntry->immediateReward_ = heuristic_(currentEntry, currentEntry->getState(),
                currentNode->getHistoricalData());
        status = SearchStatus::FINISHED;
    } else if (status == SearchStatus::FINISHED) {
        // Finished normally; no problems.
    } else if (status == SearchStatus::UNINITIALIZED) {
        debug::show_message("ERROR: Search algorithm could not initialize.");
        return status;
    } else if (status == SearchStatus::INITIAL) {
        debug::show_message("ERROR: Search algorithm initialized but did not run.");
        return status;
    } else if (status == SearchStatus::ERROR) {
        debug::show_message("ERROR: Error in search algorithm!");
        return status;
    } else {
        debug::show_message("ERROR: Invalid search status.");
        return status;
    }

    if (doBackup) {
        // Finally, we make sure to backup the sequence.
        solver_->updateSequence(sequence);
    }
    return status;
}
} /* namespace solver */
