#include "solver/search/steppers/nn_rollout.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {

NnRolloutFactory::NnRolloutFactory(Solver *solver, long maxNnComparisons, double maxNnDistance) :
            solver_(solver),
            maxNnComparisons_(maxNnComparisons),
            maxNnDistance_(maxNnDistance),
            nnMap_() {
}

BeliefNode* NnRolloutFactory::findNeighbor(BeliefNode *belief) {
    if (maxNnDistance_ < 0) {
        return nullptr;
    }
    double minDist = std::numeric_limits<double>::infinity();
    BeliefNode *nearestBelief = nnMap_[belief].neighbor;
    if (nearestBelief != nullptr) {
        minDist = belief->distL1Independent(nearestBelief);
    }

    long numTried = 0;
    for (BeliefNode *otherBelief : solver_->getPolicy()->getNodes()) {
        if (belief == otherBelief) {
            continue;
        }
        if (numTried >= maxNnComparisons_) {
            break;
        } else {
            if (nnMap_[belief].tNnComp < belief->getTimeOfLastChange()
                    || nnMap_[belief].tNnComp < otherBelief->getTimeOfLastChange()) {
                double distance = belief->distL1Independent(otherBelief);
                if (distance < minDist) {
                    minDist = distance;
                    nearestBelief = otherBelief;
                }
            }
            numTried++;
        }
    }
    nnMap_[belief].tNnComp = abt::clock_ms();
    if (minDist > maxNnDistance_) {
        return nullptr;
    }
    nnMap_[belief].neighbor = nearestBelief;
    return nearestBelief;
}

std::unique_ptr<StepGenerator> NnRolloutFactory::createGenerator(SearchStatus &status,
        HistoryEntry const *entry, State const */*state*/, HistoricalData const */*data*/) {
    return std::make_unique<NnRolloutGenerator>(status, solver_, this, entry);
}

NnRolloutGenerator::NnRolloutGenerator(SearchStatus &status, Solver *solver,
        NnRolloutFactory *factory, HistoryEntry const *entry) :
            StepGenerator(status),
            model_(solver->getModel()),
            factory_(factory),
            currentNeighborNode_(nullptr) {
    BeliefNode *currentNode = entry->getAssociatedBeliefNode();
    currentNeighborNode_ = factory_->findNeighbor(currentNode);
    if (currentNeighborNode_ == nullptr) {
        status_ = SearchStatus::UNINITIALIZED;
    } else{
        status_ = SearchStatus::INITIAL;
    }
}

Model::StepResult NnRolloutGenerator::getStep(HistoryEntry const */*entry*/, State const *state,
        HistoricalData const */*data*/) {
    if (currentNeighborNode_ == nullptr) {
        // NN rollout is done.
        status_ = SearchStatus::OUT_OF_STEPS;
        return Model::StepResult { };
    }

    std::unique_ptr<Action> action = currentNeighborNode_->getRecommendedAction();
    Model::StepResult result = model_->generateStep(*state, *action);
    currentNeighborNode_ = currentNeighborNode_->getChild(*action, *result.observation);
    return std::move(result);
}

} /* namespace solver */
