#include "NnRolloutStrategy.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/mappings/ActionMapping.hpp"

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

std::unique_ptr<SearchInstance> NnRolloutStrategy::createSearchInstance(
        HistorySequence *sequence, long maximumDepth) {
    return std::make_unique<NnRolloutInstance>(this, solver_,
            sequence, maximumDepth);
}

NnRolloutInstance::NnRolloutInstance(NnRolloutStrategy *parent,
        Solver *solver, HistorySequence *sequence, long maximumDepth) :
        AbstractSearchInstance(solver, sequence, maximumDepth),
        parent_(parent),
        rootNeighborNode_(nullptr),
        currentNeighborNode_(nullptr),
        previousAction_(nullptr) {
}

SearchStatus NnRolloutInstance::initialize() {
    rootNeighborNode_ = parent_->findNeighbor(currentNode_);
    currentNeighborNode_ = rootNeighborNode_;
    if (rootNeighborNode_ != nullptr) {
        status_ = SearchStatus::INITIAL;
    }
    return status_;
}

std::pair<SearchStatus, std::unique_ptr<Action>>
NnRolloutInstance::getStatusAndNextAction() {
    // If this is our first action, it *must* be one of the previously
    // unselected rollout actions.
    if (previousAction_ == nullptr) {
        previousAction_ = currentNode_->getMapping()->getRandomUnvisitedAction();
        return std::make_pair(SearchStatus::ROLLING_OUT, previousAction_->copy());
    }
    HistoryEntry *previousEntry = sequence_->getEntry(sequence_->getLength() - 2);
    currentNeighborNode_ = currentNeighborNode_->getChild(*previousAction_,
            *previousEntry->getObservation());

    if (currentNeighborNode_ == nullptr) {
        return std::make_pair(SearchStatus::ROLLOUT_COMPLETE, nullptr);
    }
    return std::make_pair(SearchStatus::ROLLING_OUT, currentNeighborNode_->getBestAction());
}

} /* namespace solver */
