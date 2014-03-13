#include "Model.hpp"

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/ActionPool.hpp"
#include "solver/mappings/ObservationMapping.hpp"
#include "solver/mappings/ObservationPool.hpp"

#include "solver/indexing/StateIndex.hpp"
#include "solver/indexing/RTree.hpp"

#include "solver/changes/DefaultHistoryCorrector.hpp"
#include "solver/changes/HistoryCorrector.hpp"

#include "solver/search/SearchStrategy.hpp"
#include "solver/search/UcbSearchStrategy.hpp"
#include "solver/search/RandomRolloutStrategy.hpp"

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
std::vector<long> Model::loadChanges(char const */*changeFilename*/) {
    return std::vector<long>{};
}

// Default = do nothing.
void Model::update(long /*time*/, StatePool */*pool*/) {
}

// Default = do nothing.
void Model::drawEnv(std::ostream &/*os*/) {
}

// Default = do nothing.
void Model::drawSimulationState(BeliefNode */*belief*/, State const &/*state*/,
        std::ostream &/*os*/) {
}

std::unique_ptr<StateIndex> Model::createStateIndex() {
    return std::make_unique<RTree>(getNStVars());
}

std::unique_ptr<HistoryCorrector> Model::createHistoryCorrector() {
    return std::make_unique<DefaultHistoryCorrector>(this);
}

std::unique_ptr<SearchStrategy> Model::createSearchStrategy() {
    return std::make_unique<UcbSearchStrategy>(getUcbExploreCoefficient());
}

std::unique_ptr<SearchStrategy> Model::createRolloutStrategy() {
    return std::make_unique<RandomRolloutStrategy>(2);
}
} /* namespace solver */
