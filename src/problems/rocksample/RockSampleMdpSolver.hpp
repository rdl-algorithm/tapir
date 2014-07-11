/** @file RockSampleMdpSolver.hpp
 *
 * Defines the RockSampleMdpSolver class, which solves the fully observable version of
 * RockSample in order to serve as a heuristic function for the POMDP.
 *
 * This file also contains the definition for RockSampleMdpSolver, which allows this heuristic to be
 * selected via the string "exactMdp()" in the configuration file.
 */
#ifndef ROCKSAMPLE_MDPSOLVER_HPP_
#define ROCKSAMPLE_MDPSOLVER_HPP_

#include <iostream>
#include <map>
#include <unordered_set>
#include <utility>

#include "global.hpp"

#include "problems/shared/GridPosition.hpp"
#include "problems/shared/parsers.hpp"

#include "solver/abstract-problem/heuristics/HeuristicFunction.hpp"

namespace rocksample {
class RockSampleModel;
class RockSampleState;

/** A class that solves a fully observable version of RockSample, in which it is known which rocks
 * are good and which ones are bad.
 */
class RockSampleMdpSolver {
public:
    /** Constructs a new MDP solver for the given model. */
    RockSampleMdpSolver(RockSampleModel *model);
    virtual ~RockSampleMdpSolver() = default;
    _NO_COPY_OR_MOVE(RockSampleMdpSolver);

    /** Solves the MDP again from scratch, using the current state of the model. */
    void solve();

    /** Calculates the exact value for the given state in the MDP. */
    double getQValue(RockSampleState const &state) const;

private:
    /** Calculates the q-value for the given action, from the current position
     * with the given rock state code (encoded into a number).
     *
     * The action is a simplified MDP action:
     * -1 => exit the map
     * 0+ => go to the given rock and sample it (the rock is assumed to be good)
     */
    double calculateQValue(GridPosition pos,
            long rockStateCode, long action) const;

    RockSampleModel *model_;
    std::map<std::pair<int, int>, double> valueMap_;
};

/** A class to parse the command-line heuristic setting for the case "exactMdp()". */
class RockSampleMdpParser : public shared::Parser<solver::HeuristicFunction> {
public:
    /** Creates a new MDP parser associated with the given RockSampleModel instance. */
    RockSampleMdpParser(RockSampleModel *model);
    virtual ~RockSampleMdpParser() = default;
    _NO_COPY_OR_MOVE(RockSampleMdpParser);

    /** Creates a solver::HeuristicFunction associated with the stored RockSampleModel instance.
     *
     * This heuristic can then be used by the ABT algorithm.
     */
    virtual solver::HeuristicFunction parse(solver::Solver *solver, std::vector<std::string> args);

private:
    /** The RockSampleModel instance this heuristic parser is associated with. */
    RockSampleModel *model_;
};
} /* namespace rocksample */

#endif /* ROCKSAMPLE_MDPSOLVER_HPP_ */
