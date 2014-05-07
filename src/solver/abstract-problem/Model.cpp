#include "Model.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/ActionPool.hpp"
#include "solver/mappings/ObservationMapping.hpp"
#include "solver/mappings/ObservationPool.hpp"

#include "solver/indexing/StateIndex.hpp"
#include "solver/indexing/RTree.hpp"

#include "solver/backpropagation/BackpropagationStrategy.hpp"
#include "solver/backpropagation/AveragePropagator.hpp"

#include "solver/changes/DefaultHistoryCorrector.hpp"
#include "solver/changes/HistoryCorrector.hpp"

#include "solver/search/HistoricalData.hpp"
#include "solver/search/SearchStrategy.hpp"
#include "solver/search/UcbSelectionStrategy.hpp"
#include "solver/search/DefaultRolloutStrategy.hpp"

#include "Action.hpp"        // for Action
#include "State.hpp"                    // for State
#include "Observation.hpp"              // for Observation
#include "TransitionParameters.hpp"

namespace solver {
std::unique_ptr<TransitionParameters> Model::generateTransition(
        State const &/*state*/,
        Action const &/*action*/) {
    return nullptr;
}

std::vector<std::unique_ptr<State>> Model::generateParticles(
        BeliefNode *previousBelief,
        Action const &action, Observation const &obs,
        std::vector<State const *> const &previousParticles) {
    std::vector<std::unique_ptr<State>> particles;
    ObservationMapping *obsMap = previousBelief->getMapping()->getActionNode(
            action)->getMapping();
    BeliefNode *childNode = obsMap->getBelief(obs);
    while (particles.size() < getNParticles()) {
        long index = std::uniform_int_distribution<long>(0,
                previousParticles.size() - 1)(*getRandomGenerator());
        State const *state = previousParticles[index];
        StepResult result = generateStep(*state, action);
        if (obsMap->getBelief(*result.observation) == childNode) {
            particles.push_back(std::move(result.action));
        }
    }
    return particles;
}

std::vector<std::unique_ptr<State>> Model::generateParticles(
        BeliefNode *previousBelief,
        Action const &action, Observation const &obs) {
    std::vector<std::unique_ptr<State>> particles;
    ObservationMapping *obsMap = previousBelief->getMapping()->getActionNode(
            action)->getMapping();
    BeliefNode *childNode = obsMap->getBelief(obs);
    while (particles.size() < getNParticles()) {
        std::unique_ptr<State> state = sampleStateUniform();
        StepResult result = generateStep(*state, action);
        if (obsMap->getBelief(*result.observation) == childNode) {
            particles.push_back(std::move(result.action));
        }
    }
    return particles;
}

// Default = no changes.
void Model::applyChange(ModelChange const &/*change*/, StatePool */*pool*/) {
}

std::unique_ptr<StateIndex> Model::createStateIndex() {
    return std::make_unique<RTree>(getNumberOfStateVariables());
}

std::unique_ptr<HistoryCorrector> Model::createHistoryCorrector(Solver *solver) {
    return std::make_unique<DefaultHistoryCorrector>(solver);
}

std::unique_ptr<SearchStrategy> Model::createSelectionStrategy(Solver *solver) {
    return std::make_unique<UcbSelectionStrategy>(solver, 1.0);
}

std::unique_ptr<SearchStrategy> Model::createRolloutStrategy(Solver *solver) {
    return std::make_unique<DefaultRolloutStrategy>(solver, 1);
}

std::unique_ptr<BackpropagationStrategy> Model::createBackpropagationStrategy(
            Solver *solver) {
    return std::make_unique<AveragePropagator>(solver);
}

std::unique_ptr<HistoricalData> Model::createRootHistoricalData() {
    return nullptr;
}

/* --------------- Pretty printing methods ----------------- */
// Default = do nothing.
void Model::drawEnv(std::ostream &/*os*/) {
}
void Model::drawSimulationState(BeliefNode */*belief*/, State const &/*state*/,
        std::ostream &/*os*/) {
}
std::string Model::getName() {
    return "Default Model";
}
bool Model::hasColorOutput() {
    return false;
}
bool Model::hasVerboseOutput() {
    return false;
}
} /* namespace solver */
