#include "solver/abstract-problem/BasicModel.hpp"

namespace solver {
double BasicModel::getHeuristicValue(HistoricalData const */*data*/,
        State const */*state*/) {
    return 0;
}

/** Allows for a basic rollout strategy based on */
virtual std::unique_ptr<Action> getRolloutAction(HistoricalData const *data,
        State const *state);

virtual std::unique_ptr<ActionPool> BasicModel::createActionPool(Solver *solver) {
    return std::make_unique<EnumeratedActionPool

}

virtual std::unique_ptr<ObservationPool> BasicModel::createObservationPool(Solver *solver) {
    return std::make_unique<DiscreteObservationPool>();
}

/* ------- Customization of more complex solver functionality  --------- */
std::unique_ptr<SearchStrategy> BasicModel::createSearchStrategy(Solver *solver) {
    using namespace std::placeholders;
    return std::make_unique<BasicSearchStrategy>(solver,
            std::make_unique<UcbStepGeneratorFactory>(solver, 1.0),
            std::make_unique<DefaultHeuristic>(this));
}
} /* namespace solver */
