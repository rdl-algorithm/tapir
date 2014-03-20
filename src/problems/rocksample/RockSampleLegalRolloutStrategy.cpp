#include "RockSampleLegalRolloutStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"

#include "RockSampleAction.hpp"
#include "RockSampleModel.hpp"
#include "RockSampleState.hpp"

namespace rocksample {

RockSampleLegalRolloutStrategy::RockSampleLegalRolloutStrategy(
        solver::Solver *solver, long maxNSteps) :
        solver::SearchStrategy(solver),
        maxNSteps_(maxNSteps) {
}

std::unique_ptr<solver::SearchInstance> RockSampleLegalRolloutStrategy::createSearchInstance(
                solver::HistorySequence *sequence,
                long maximumDepth) {
    return std::make_unique<RockSampleLegalRolloutInstance>(maxNSteps_, solver_,
            sequence, maximumDepth);
}

RockSampleLegalRolloutInstance::RockSampleLegalRolloutInstance(long maxNSteps,
        solver::Solver *solver, solver::HistorySequence *sequence,
        long maximumDepth) :
                solver::AbstractSearchInstance(solver, sequence, maximumDepth),
                maxNSteps_(maxNSteps),
                currentNSteps_(0) {
}

std::pair<solver::SearchStatus, std::unique_ptr<solver::Action>>
RockSampleLegalRolloutInstance::getStatusAndNextAction() {
    if (currentNSteps_ >= maxNSteps_) {
        return std::make_pair(solver::SearchStatus::ROLLOUT_COMPLETE, nullptr);
    }
    currentNSteps_++;
    std::vector<std::unique_ptr<solver::Action>> rolloutActions = currentNode_->getMapping()->getRolloutActions();
    RockSampleModel &model = dynamic_cast<RockSampleModel &>(*model_);
    RockSampleState const &state = static_cast<RockSampleState const &>(
            *sequence_->getLastEntry()->getState());

    std::vector<RockSampleAction const *> legalActions;
    for (std::unique_ptr<solver::Action> &action : rolloutActions) {
        RockSampleAction const &rsAction = static_cast<RockSampleAction const &>(*action);

        if (model.makeNextState(state, rsAction).second) {
            legalActions.push_back(&rsAction);
        }
    }

    // If there's no legal action we still pick an illegal one...
    if (legalActions.empty()) {
        return std::make_pair(solver::SearchStatus::ROLLING_OUT,
                currentNode_->getMapping()->getRandomRolloutAction());
    }

    // Otherwise, we sample one of the legal actions at random.
    int index = std::uniform_int_distribution<int>(0, legalActions.size() - 1)(
            *model.getRandomGenerator());
    return std::make_pair(solver::SearchStatus::ROLLING_OUT,
            legalActions[index]->copy());
}

std::unique_ptr<solver::SearchStrategy> RockSampleLegalParser::parseStrategy(
            solver::Solver *solver, AllStrategiesParser */*allParser*/,
            std::vector<std::string> args) {
    long nSteps;
    std::istringstream(args[0]) >> nSteps;
    return std::make_unique<RockSampleLegalRolloutStrategy>(solver, nSteps);
}

} /* namespace rocksample */
