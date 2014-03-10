#include "NnRolloutStrategy.hpp"

namespace solver {

NnRolloutStrategy::NnRolloutStrategy(long maxNnComparisons,
        double maxNnDistance) :
        maxNnComparisons_(maxNnComparisons),
        maxNnDistance_(maxNnDistance) {
}

NnRolloutInstance::NnRolloutInstance(long maxNnComparisons,
        double maxNnDistance,
        Solver *solver,
        BeliefNode *currentNode, HistorySequence *sequence,
        double discountFactor, long maximumDepth) :
    SearchInstance(solver, currentNode, sequence, discountFactor, maximumDepth) {
        neighborNode_ = solver_->getNNBelNode(currentNode);


}

NnRolloutInstance::getNextAction() {
    std::vector<ActionNode *> actionChildren = currentNode_->getMapping()->getChildren();
    for (ActionNode *actionNode : actionChildren) {

    }
}

} /* namespace solver */
