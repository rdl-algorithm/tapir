#include "solver/abstract-problem/BasicModel.hpp"

namespace solver {
virtual std::unique_ptr<ActionPool> BasicModel::createActionPool(Solver *solver) {
    return std::make_unique<

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
