/** @file TagMdpSolver.hpp
 *
 * Defines the TagMdpSolver class, which solves the fully observable version of Tag in order to
 * serve as a heuristic function for the POMDP.
 *
 * This file also contains the definition for TagMdpParser, which allows this heuristic to be
 * selected via the string "exactMdp()" in the configuration file.
 */
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

/** A class that solves the fully observable version of Tag and stores the calculated value for
 * each state.
 */
class TagMdpSolver {
public:
    /** Creates a new TagMdpSolver which will be tied to the given TagModel instance. */
    TagMdpSolver(TagModel *model);
    virtual ~TagMdpSolver() = default;
    _NO_COPY_OR_MOVE(TagMdpSolver);

    /** Solves the MDP, using the current state of the */
    void solve();

    /** Returns the calculated MDP value for the given state. */
    double getValue(TagState const &state) const;

private:
    /** The model instance this MDP solver is associated with. */
    TagModel *model_;
    /** A map to hold the calculated value for each non-terminal state. */
    std::unordered_map<TagState, double> valueMap_;
};

/** A class to parse the command-line heuristic setting for the case "exactMdp()". */
class TagMdpParser : public shared::Parser<solver::HeuristicFunction> {
public:
    /** Creates a new MDP parser associated with the given TagModel instance. */
    TagMdpParser(TagModel *model);
    virtual ~TagMdpParser() = default;
    _NO_COPY_OR_MOVE(TagMdpParser);

    /** Creates a solver::HeuristicFunction associated with the stored TagModel instance.
     *
     * This heuristic can then be used by the ABT algorithm.
     */
    virtual solver::HeuristicFunction parse(solver::Solver *solver, std::vector<std::string> args);

private:
    /** The TagModel instance this heuristic parser is associated with. */
    TagModel *model_;
};
} /* namespace tag */

#endif /* TAG_MDPSOLVER_HPP_ */
