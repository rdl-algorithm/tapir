#ifndef SOLVER_OPTIONS_HPP_
#define SOLVER_OPTIONS_HPP_

#include "options/option_parser.hpp"

namespace solver {
struct Options : options::BaseOptions {
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
    /** The minimum number of particles to maintain in the active belief node. */
    unsigned long minParticleCount = 1000;
    /** The number of new histories to generate on each search step. */
    unsigned long historiesPerStep = 1000;
    /** The maximum time (in milliseconds) to spend on each search step. */
    double stepTimeout = 1000;
    /** The maximum allowable depth in the search tree. */
    long maximumDepth = 100;

    /* ----------------------- ABT output modes ------------------- */
    /** True iff color output is allowed. */
    bool hasColorOutput = false;
    /** True iff verbose output is enabled. */
    bool hasVerboseOutput = false;
};
} /* namespace solver */

#endif /* SOLVER_OPTIONS_HPP_ */
