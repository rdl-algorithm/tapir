#include "ucb_search.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/action-choosers/choosers.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {
UcbStepGeneratorFactory::UcbStepGeneratorFactory(Solver *solver, double explorationCoefficient) :
            solver_(solver),
            explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<StepGenerator> UcbStepGeneratorFactory::createGenerator(SearchStatus &status,
        HistorySequence *sequence) {
    return std::make_unique<UcbStepGenerator>(status, sequence, solver_, explorationCoefficient_);
}

UcbStepGenerator::UcbStepGenerator(SearchStatus &status, HistorySequence *sequence, Solver *solver,
        double explorationCoefficient) :
            StepGenerator(status),
            sequence_(sequence),
            solver_(solver),
            model_(solver_->getModel()),
            explorationCoefficient_(explorationCoefficient),
            choseUnvisitedAction_(false) {
    status_ = SearchStatus::INITIAL;
}

Model::StepResult UcbStepGenerator::getStep() {
    if (choseUnvisitedAction_) {
        // We've reached the new leaf node - this search is over.
        return Model::StepResult { };
    }
    HistoryEntry *currentEntry = sequence_->getLastEntry();
    BeliefNode *currentNode = sequence_->getLastEntry()->getAssociatedBeliefNode();

    ActionMapping *mapping = currentNode->getMapping();

    State const &currentState = *currentEntry->getState();
    std::unique_ptr<Action> action;
    if (mapping->hasUnvisitedActions()) {
        // If there are unvisited actions, we take one, and we're finished with UCB.
        choseUnvisitedAction_ = true;
        action = mapping->getRandomUnvisitedAction();
    } else {
        // Use UCB to get the best action.
        action = choosers::ucb_action(currentNode, explorationCoefficient_);
    }

    // NO action -> error!
    if (action == nullptr) {
        debug::show_message("ERROR: node has no actions!?");
        status_ = SearchStatus::ERROR;
        return Model::StepResult { };
    }

    return model_->generateStep(currentState, *action);
}
} /* namespace solver */
