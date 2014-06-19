#ifndef SHAREDOPTIONS_HPP_
#define SHAREDOPTIONS_HPP_

// #include "global.hpp"

#include "solver/abstract-problem/Options.hpp"

namespace shared {
struct SharedOptions: public solver::Options {
    /* ---------------------- Generic settings  ------------------ */
    /** The path to the configuration file. */
    std::string configPath;
    /** The path to the policy file. */
    std::string policyPath;
    /** The seed value to use for the RNG. */
    unsigned long seed;
    /** A custom state to load for RNG. */
    unsigned long rngState;

    /* --------------------- Simulation settings  ----------------- */
    /** The path to the log file. */
    std::string logPath;
    /** The maximum number of steps to simulate per run. */
    long nSimulationSteps;
    /** The number of simulations to run. */
    long nRuns;
    /** True iff we should save the resulting policy at the end of each simulation. */
    bool savePolicy;

    /* ----------------- Simulation settings: changes  ------------ */
    /** True iff there are pre-planned model changed during the simulation. */
    bool hasChanges;
    /** True iff the changes are dynamic. */
    bool areDynamic;
    /** The path to the change file. */
    std::string changesPath;

    /* ---------- ABT settings: advanced customization  ---------- */
    /** The heuristic used for searches. */
    std::string searchHeuristic;
    /* The search strategy to use. */
    std::string searchStrategy;
    /* The function to estimate the value of a belief. */
    std::string estimator;
    /* The maximum distance between observations to group together; only applicable if
     * approximate observations are in use. */
    double maxObservationDistance;
};
} /* namespace shared */

#endif /* SHAREDOPTIONS_HPP_ */
