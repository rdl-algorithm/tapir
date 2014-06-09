#ifndef SOLVER_HEURISTICS_HPP_
#define SOLVER_HEURISTICS_HPP_

#include "global.hpp"

#include "solver/abstract-problem/Model.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/HistoricalData.hpp"

namespace solver {
class HistoryEntry;
class Model;
class Solver;

typedef std::function<double(HistoryEntry const *, State const *, HistoricalData const *)> Heuristic;

namespace heuristics {
double default_(HistoryEntry const *entry, State const *state, HistoricalData const *data, Model *model);

Heuristic get_default_heuristic(Model *model);

double zero(HistoryEntry const *entry, State const *state, HistoricalData const *data);
} /* namespace heuristics */
} /* namespace solver */

#endif /* SOLVER_HEURISTICS_HPP_ */
