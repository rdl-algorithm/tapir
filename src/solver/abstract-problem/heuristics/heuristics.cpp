#include "solver/search/heuristics/heuristics.hpp"

#include "solver/abstract-problem/Model.hpp"

namespace solver {
double ZeroHeuristic::getHeuristicValue(HistoryEntry const */*entry*/,
        State const */*state*/, HistoricalData const */*data*/) {
    return 0;
}

DefaultHeuristic::DefaultHeuristic(Model *model) :
        model_(model) {
}
double DefaultHeuristic::getHeuristicValue(HistoryEntry const */*entry*/,
        State const *state, HistoricalData const */*data*/) {
    return model_->getHeuristicValue(*state);
}
} /* namespace solver */
