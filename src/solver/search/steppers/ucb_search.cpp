#include "solver/search/steppers/ucb_search.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/search/action-choosers/choosers.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {
UcbStepGeneratorFactory::UcbStepGeneratorFactory(Solver *solver, double explorationCoefficient) :
            solver_(solver),
            explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<StepGenerator> UcbStepGeneratorFactory::createGenerator(SearchStatus &status,
        HistoryEntry const */*entry*/, State const */*state*/, HistoricalData const */*data*/) {
    return std::make_unique<UcbStepGenerator>(status, solver_, explorationCoefficient_);
}

UcbStepGenerator::UcbStepGenerator(SearchStatus &status, Solver *solver,
        double explorationCoefficient) :
            StepGenerator(status),
            model_(solver->getModel()),
            explorationCoefficient_(explorationCoefficient),
            choseUnvisitedAction_(false) {
    status_ = SearchStatus::INITIAL;
}

Model::StepResult UcbStepGenerator::getStep(HistoryEntry const *entry, State const *state,
        HistoricalData const */*data*/) {
    if (choseUnvisitedAction_) {
        // We've reached the new leaf node - this search is over.
        status_ = SearchStatus::OUT_OF_STEPS;
        return Model::StepResult { };
    }
    BeliefNode *currentNode = entry->getAssociatedBeliefNode();
    ActionMapping *mapping = currentNode->getMapping();

    std::unique_ptr<Action> action = mapping->getNextActionToTry();
    if (action != nullptr) {
        // If there are unvisited actions, we take one, and we're finished with UCB search.
        choseUnvisitedAction_ = true;
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

    return model_->generateStep(*state, *action);
}
} /* namespace solver */
