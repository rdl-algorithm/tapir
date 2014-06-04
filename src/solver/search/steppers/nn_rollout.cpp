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
        HistorySequence *sequence) {
    return std::make_unique<NnRolloutGenerator>(status, sequence, solver_, this);
}

NnRolloutGenerator::NnRolloutGenerator(SearchStatus &status, HistorySequence *sequence,
        Solver *solver, NnRolloutFactory *factory) :
            StepGenerator(status),
            sequence_(sequence),
            solver_(solver),
            model_(solver_->getModel()),
            factory_(factory),
            rootNeighborNode_(nullptr),
            currentNeighborNode_(nullptr),
            previousAction_(nullptr) {
    BeliefNode *currentNode = sequence_->getLastEntry()->getAssociatedBeliefNode();
    rootNeighborNode_ = factory_->findNeighbor(currentNode);
    currentNeighborNode_ = rootNeighborNode_;
    if (rootNeighborNode_ == nullptr) {
        status_ = SearchStatus::ERROR;
    }
}

Model::StepResult NnRolloutGenerator::getStep() {
    if (currentNeighborNode_ == nullptr) {
        // NN rollout is done.
        status_ = SearchStatus::STAGE_COMPLETE;
        return Model::StepResult { };
    }

    HistoryEntry *lastEntry = sequence_->getLastEntry();
    std::unique_ptr<Action> action = currentNeighborNode_->getRecommendedAction();
    Model::StepResult result = model_->generateStep(*lastEntry->getState(), *action);
    currentNeighborNode_ = currentNeighborNode_->getChild(*action, *result.observation);

    return std::move(result);
}

} /* namespace solver */
