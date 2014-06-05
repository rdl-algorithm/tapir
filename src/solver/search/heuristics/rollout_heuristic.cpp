#include "solver/search/heuristics/rollout_heuristic.hpp"

#include "solver/HistoryEntry.hpp"
#include "solver/abstract-problem/Model.hpp"

namespace solver {
RolloutHeuristic::RolloutHeuristic(std::unique_ptr<StepGeneratorFactory> factory) :
        factory_(std::move(factory)) {
}
double RolloutHeuristic::getHeuristicValue(HistoryEntry const */*entry*/,
        State const *state, HistoricalData const */*data*/) {

}
} /* namespace solver */
