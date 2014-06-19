#ifndef SOLVER_OPTIONS_HPP_
#define SOLVER_OPTIONS_HPP_

// #include "global.hpp"

namespace solver {
struct Options {
    /* ------------------- Model-specific settings  --------------- */
    /** The name of this problem. */
    std::string problemName;
    /** The number of state variables for this problem. */
    long numberOfStateVariables;
    /** A default lower bound on the value function. */
    double minVal;
    /** A default upper bound on the value function. */
    double maxVal;

    /* ------------------- Generic POMDP parameters --------------- */
    /** The discount factor of the PODMP. */
    double discountFactor;

    /* ------------------------- ABT settings --------------------- */
    /** The minimum number of particles to maintain in the active belief node. */
    unsigned long minParticleCount;
    /** The number of new histories to generate on each search step. */
    unsigned long historiesPerStep;
    /** The maximum time (in milliseconds) to spend on each search step. */
    double stepTimeout;
    /** The maximum allowable depth in the search tree. */
    long maximumDepth;

    /* ----------------------- ABT output modes ------------------- */
    /** True iff color output is allowed. */
    bool hasColorOutput;
    /** True iff verbose output is enabled. */
    bool hasVerboseOutput;
};
} /* namespace solver */

#endif /* SOLVER_OPTIONS_HPP_ */
