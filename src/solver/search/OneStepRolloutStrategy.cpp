#include "UcbSearchStrategy.hpp"

namespace solver {

UcbSearchStrategy::UcbSearchStrategy(double explorationCoefficient) :
        explorationCoefficient_(explorationCoefficient) {
}

UcbSearchInstance::UcbSearchInstance(double explorationCoefficient,
        Solver *solver,
        BeliefNode *startNode, HistorySequence *sequence,
        double discountFactor, long maximumDepth) :
    SearchInstance(solver, startNode, sequence, discountFactor, maximumDepth),
    explorationCoefficient_(explorationCoefficient) {
}

UcbSearchInstance::getNextAction() {
    std::vector<ActionNode *> actionChildren = currentNode_->getMapping()->getChildren();
    for (ActionNode *actionNode : actionChildren) {

    }
}

} /* namespace solver */
