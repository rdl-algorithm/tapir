#ifndef SOLVER_ROLLOUT_HEURISTIC_HPP_
#define SOLVER_ROLLOUT_HEURISTIC_HPP_

#include "global.hpp"

#include <memory>

#include "solver/abstract-problem/heuristics/heuristics.hpp"

#include "solver/search/search_interface.hpp"

namespace solver {
class HistoryEntry;

class RolloutHeuristic {
public:
    RolloutHeuristic(Model *model, std::unique_ptr<StepGeneratorFactory> factory,
            Heuristic heuristic);
    virtual ~RolloutHeuristic() = default;
    _NO_COPY_OR_MOVE(RolloutHeuristic);

    virtual double getHeuristicValue(HistoryEntry const *entry,
            State const *state, HistoricalData const *data);
private:
    Model *model_;
    std::unique_ptr<StepGeneratorFactory> factory_;
    Heuristic heuristic_;

};
} /* namespace solver */

#endif /* SOLVER_ROLLOUT_HEURISTIC_HPP_ */
