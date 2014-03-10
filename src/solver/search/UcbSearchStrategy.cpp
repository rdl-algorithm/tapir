#include "UcbSearchStrategy.hpp"

namespace solver {

UcbSearchStrategy::UcbSearchStrategy(double explorationCoefficient) :
        explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<SearchInstance> UcbSearchStrategy::createSearchInstance(
       Solver *solver,
       BeliefNode *currentNode, HistorySequence *sequence,
       double discountFactor, long maximumDepth) {
    return std::make_unique<UcbSearchInstance>(solver, currentNode,
            sequence, discountFactor, maximumDepth);
}

UcbSearchInstance::UcbSearchInstance(double explorationCoefficient,
        Solver *solver,
        BeliefNode *currentNode, HistorySequence *sequence,
        double discountFactor, long maximumDepth) :
    SearchInstance(solver, currentNode, sequence, discountFactor, maximumDepth),
    explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<Action> UcbSearchInstance::getNextAction() {
    // TODO Implement UCB action selection.
    return nullptr;
}

} /* namespace solver */
