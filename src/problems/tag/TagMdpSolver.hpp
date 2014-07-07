#ifndef TAG_MDPSOLVER_HPP_
#define TAG_MDPSOLVER_HPP_

#include <iostream>
#include <unordered_map>
#include <vector>

#include "global.hpp"

#include "problems/shared/parsers.hpp"

#include "solver/abstract-problem/heuristics/HeuristicFunction.hpp"

#include "TagState.hpp"

namespace tag {
class TagModel;

/** Solves an MDP version of Tag */
class TagMdpSolver {
public:
    TagMdpSolver(TagModel *model);
    virtual ~TagMdpSolver() = default;
    _NO_COPY_OR_MOVE(TagMdpSolver);

    /** Solves the MDP. */
    void solve();

    /** Calculates the exact value for the given state. */
    double getQValue(TagState const &state) const;

private:
    TagModel *model_;
    std::unordered_map<TagState, double> valueMap_;
};

class TagMdpParser : public shared::Parser<solver::HeuristicFunction> {
public:
    TagMdpParser(TagModel *model);
    virtual ~TagMdpParser() = default;
    _NO_COPY_OR_MOVE(TagMdpParser);

    virtual solver::HeuristicFunction parse(solver::Solver *solver, std::vector<std::string> args);
private:
    TagModel *model_;
};
} /* namespace tag */

#endif /* TAG_MDPSOLVER_HPP_ */
