#include "NnRolloutStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {

NnRolloutStrategy::NnRolloutStrategy(Solver *solver,
        long maxNnComparisons, double maxNnDistance) :
                SearchStrategy(solver),
        maxNnComparisons_(maxNnComparisons),
        maxNnDistance_(maxNnDistance),
        nnMap_() {
}

BeliefNode* NnRolloutStrategy::findNeighbor(BeliefNode *belief) {
    if (maxNnDistance_ < 0) {
        return nullptr;
    }
    double minDist = std::numeric_limits<double>::infinity();
    BeliefNode *nearestBelief = nnMap_[belief].neighbor;
    if (nearestBelief != nullptr) {
        minDist = belief->distL1Independent(nearestBelief);
    }

    long numTried = 0;
    for (BeliefNode *otherBelief : getSolver()->getPolicy()->getNodes()) {
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

std::unique_ptr<SearchInstance> NnRolloutStrategy::createSearchInstance(
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<NnRolloutInstance>(this, getSolver(),
            sequence, maximumDepth);
}

NnRolloutInstance::NnRolloutInstance(NnRolloutStrategy *strategy,
        Solver *solver, HistorySequence *sequence, long maximumDepth) :
        AbstractRolloutInstance(solver, sequence, maximumDepth),
        strategy_(strategy),
        rootNeighborNode_(nullptr),
        currentNeighborNode_(nullptr),
        previousAction_(nullptr) {
}

SearchStatus NnRolloutInstance::initialize(BeliefNode *currentNode) {
    rootNeighborNode_ = strategy_->findNeighbor(currentNode);
    currentNeighborNode_ = rootNeighborNode_;
    if (rootNeighborNode_ != nullptr) {
        return SearchStatus::INITIAL;
    }
    return SearchStatus::UNINITIALIZED;
}

SearchStep NnRolloutInstance::getSearchStep(HistoricalData */*historicalData*/) {
    HistorySequence *sequence = getSequence();
    HistoryEntry *previousEntry = sequence->getEntry(sequence->getLength() - 2);
    currentNeighborNode_ = currentNeighborNode_->getChild(*previousAction_,
            *previousEntry->getObservation());

    if (currentNeighborNode_ == nullptr) {
        return SearchStep {SearchStatus::ROLLOUT_COMPLETE, nullptr, false};
    }
    return SearchStep {SearchStatus::ROLLING_OUT, currentNeighborNode_->getRecommendedAction(), false};
}

} /* namespace solver */
