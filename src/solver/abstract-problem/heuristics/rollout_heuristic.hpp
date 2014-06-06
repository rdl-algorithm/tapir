#ifndef SOLVER_ROLLOUT_HEURISTIC_HPP_
#define SOLVER_ROLLOUT_HEURISTIC_HPP_

#include "global.hpp"

#include "solver/abstract-problem/heuristics/heuristics.hpp"

#include "solver/search/search_interface.hpp"

namespace solver {
class HistoryEntry;

class RolloutHeuristic : Heuristic {
public:
    RolloutHeuristic(std::unique_ptr<StepGeneratorFactory> factory);
    virtual ~RolloutHeuristic() = default;

    virtual double getHeuristicValue(HistoryEntry const */*entry*/,
            State const *state, HistoricalData const */*data*/) override;
private:
    std::unique_ptr<StepGeneratorFactory> factory_;
};
} /* namespace solver */

#endif /* SOLVER_ROLLOUT_HEURISTIC_HPP_ */
