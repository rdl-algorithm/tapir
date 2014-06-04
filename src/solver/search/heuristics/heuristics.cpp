#include "solver/search/heuristics/heuristics.hpp"

#include "solver/HistoryEntry.hpp"
#include "solver/abstract-problem/Model.hpp"

namespace solver {
double ZeroHeuristic::getHeuristicValue(HistoryEntry const */*entry*/) {
    return 0;
}

DefaultHeuristic::DefaultHeuristic(Model *model) :
        model_(model) {
}
double DefaultHeuristic::getHeuristicValue(HistoryEntry const *entry) {
    return model_->getHeuristicValue(*entry->getState());
}
} /* namespace solver */
