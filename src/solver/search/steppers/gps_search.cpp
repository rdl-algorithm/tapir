/** @file gps_search.cpp
 *
 * Contains the basic implementation code for a SearchStrategy using GPS
 * search.
 */
#include "solver/search/steppers/gps_search.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/search/action-choosers/choosers.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {
GpsStepGenerator::GpsStepGenerator(SearchStatus &status, Solver *solver, choosers::GpsChooserOptions theOptions) :
            StepGenerator(status),
            model(solver->getModel()),
            options(theOptions),
            choseUnvisitedAction(false) {
    status_ = SearchStatus::INITIAL;
}

Model::StepResult GpsStepGenerator::getStep(HistoryEntry const *entry, State const *state, HistoricalData const */*data*/) {

    // If we previously chose a new action that hadn't been tried before, the search is over.
    if (choseUnvisitedAction) {
        // We've reached the new leaf node - this search is over.
        status_ = SearchStatus::OUT_OF_STEPS;
        return Model::StepResult { };
    }

    // Retrieve the mapping.
    BeliefNode *currentNode = entry->getAssociatedBeliefNode();
    //ActionMapping *mapping = currentNode->getMapping();

    choosers::GpsChooserResponse chooserResponse = choosers::gps_ucb_action(currentNode, *model, options);

    if (!chooserResponse.actionIsVisited) {
    	choseUnvisitedAction = true;
    }

    // NO action -> error!
    if (chooserResponse.action == nullptr) {
        debug::show_message("ERROR: node has no actions!?");
        status_ = SearchStatus::ERROR;
        return Model::StepResult { };
    }

    // Use the model to generate the step.
    return model->generateStep(*state, *(chooserResponse.action) );

}

GpsStepGeneratorFactory::GpsStepGeneratorFactory(Solver *theSolver, choosers::GpsChooserOptions theOptions) :
            solver(theSolver),
            options(theOptions) {
}

std::unique_ptr<StepGenerator> GpsStepGeneratorFactory::createGenerator(SearchStatus &status, HistoryEntry const */*entry*/, State const */*state*/, HistoricalData const */*data*/) {
    return std::make_unique<GpsStepGenerator>(status, solver, options);
}
} /* namespace solver */
