#include "solver/abstract-problem/heuristics/heuristics.hpp"

#include "solver/abstract-problem/Model.hpp"

namespace solver {
namespace heuristics {
double default_(HistoryEntry const */*entry*/, State const *state, HistoricalData const *data,
        Model *model) {
    return model->getHeuristicValue(data, state);
}

Heuristic get_default_heuristic(Model *model) {
    using namespace std::placeholders;
    return std::bind(default_, _1, _2, _3, model);
}

double zero(HistoryEntry const */*entry*/, State const */*state*/, HistoricalData const */*data*/) {
    return 0;
}
} /* namespace heuristics */
} /* namespace solver */
