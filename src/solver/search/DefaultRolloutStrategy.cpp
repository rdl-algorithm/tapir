#include "DefaultRolloutStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/Solver.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/mappings/actions/ActionPool.hpp"
#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {

DefaultRolloutGenerator::DefaultRolloutGenerator(HistorySequence *sequence, Solver *solver,
        long maxNSteps) :
            sequence_(sequence),
            solver_(solver),
            maxNSteps_(maxNSteps),
            currentNSteps_(0){
}

Model::StepResult DefaultRolloutGenerator::getStep() {
    Model::StepResult result;
    if (currentNSteps_ >= maxNSteps_) {
        // No action!
        return result;
    }
    currentNSteps_++;
    HistoricalData *currentData = sequence_->getLastEntry()->getAssociatedBeliefNode()->getHistoricalData();
    result.action = solver_->getActionPool()->getDefaultRolloutAction(currentData);
    return result.action;
}

DefaultRolloutStrategy::DefaultRolloutStrategy(Solver *solver, long maxNSteps) :
            SearchStrategy(solver),
            maxNSteps_(maxNSteps) {
}

std::function<

std::unique_ptr<SearchInstance> DefaultRolloutStrategy::createSearchInstance(SearchStatus &status,
                    HistorySequence *sequence, long maximumDepth) {
    std::unique_ptr<DefaultRolloutGenerator> generator = std::make_unique<DefaultRolloutGenerator>(
            sequence, getSolver(), maxNSteps_);
    return std::make_unique<BasicSearchInstance>(status, sequence, maximumDepth, getSolver(),
            std::move(generator), )

}

} /* namespace solver */
