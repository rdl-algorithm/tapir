#ifndef SOLVER_HEURISTICS_HPP_
#define SOLVER_HEURISTICS_HPP_

#include "global.hpp"

#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/HistoricalData.hpp"

namespace solver {
class HistoryEntry;
class Model;
class Solver;

class Heuristic {
public:
    Heuristic() = default;
    virtual ~Heuristic() = default;

    virtual double getHeuristicValue(HistoryEntry const *entry,
            State const *state, HistoricalData const *data) = 0;
};

class ZeroHeuristic : public Heuristic {
public:
    virtual double getHeuristicValue(HistoryEntry const *entry,
            State const *state, HistoricalData const *data) override;
};

class DefaultHeuristic : public Heuristic {
public:
    DefaultHeuristic(Model *model);
    virtual ~DefaultHeuristic() = default;
    _NO_COPY_OR_MOVE(DefaultHeuristic);

    virtual double getHeuristicValue(HistoryEntry const *entry,
            State const *state, HistoricalData const *data) override;
private:
    Model *model_;
};
} /* namespace solver */

#endif /* SOLVER_HEURISTICS_HPP_ */
