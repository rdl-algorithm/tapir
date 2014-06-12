#ifndef SOLVER_HEURISTICS_HPP_
#define SOLVER_HEURISTICS_HPP_

#include <functional>

#include "global.hpp"

#include "solver/abstract-problem/State.hpp"

namespace solver {
class HistoricalData;
class HistoryEntry;

typedef std::function<double(HistoryEntry const *, State const *, HistoricalData const *)> Heuristic;

} /* namespace solver */

#endif /* SOLVER_HEURISTICS_HPP_ */
