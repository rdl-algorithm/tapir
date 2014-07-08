/** @file Options.hpp
 *
 * Defines the base Options class. These are the core parameters that ABT requires in order to
 * function correctly.
 */
#ifndef SOLVER_OPTIONS_HPP_
#define SOLVER_OPTIONS_HPP_

#include "options/option_parser.hpp"

namespace solver {
/** The base Options class for the ABT solver.
 *
 * This is the data structure into which options are parsed; it is also used by the Model, Solver
 * and Simulator to access the configuration settings, e.g. whether or not verbose output should
 * be printed.
 *
 * The values of numberOfStateVariables, minVal, and maxVal should be set by the model to
 * correctly reflect the specific problem.
 *
 * Additional configuration settings can be added by inheriting from this class.
 */
struct Options : options::BaseOptions {
    Options() = default;
    virtual ~Options() = default;

    /* ------------------- Model-specific settings  --------------- */
    /** The number of state variables for this problem. */
    long numberOfStateVariables = -1;
    /** A default lower bound on the value function. */
    double minVal = -std::numeric_limits<double>::infinity();
    /** A default upper bound on the value function. */
    double maxVal = +std::numeric_limits<double>::infinity();

    /* ------------------- Generic POMDP parameters --------------- */
    /** The discount factor of the PODMP. */
    double discountFactor = 1.0;

    /* ------------------------- ABT settings --------------------- */
    /** Whether to prune the tree on every simulation step. */
    bool pruneEveryStep = false;
    /** The minimum number of particles to maintain in the active belief node. */
    unsigned long minParticleCount = 1000;
    /** The number of new histories to generate on each search step. */
    unsigned long historiesPerStep = 1000;
    /** The maximum time (in milliseconds) to spend on each search step. */
    double stepTimeout = 1000;
    /** The maximum depth to search, relative to the current belief node. */
    long maximumDepth = 100;

    /* ----------------------- TAPIR output modes ------------------- */
    /** True iff color output is allowed. */
    bool hasColorOutput = false;
    /** True iff verbose output is enabled. */
    bool hasVerboseOutput = false;
};
} /* namespace solver */

#endif /* SOLVER_OPTIONS_HPP_ */
