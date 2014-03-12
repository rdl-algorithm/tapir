#ifndef PROGRAMOPTIONS_HPP_
#define PROGRAMOPTIONS_HPP_

#include <sstream>                      // for basic_stringbuf<>::int_type, basic_stringbuf<>::pos_type, basic_stringbuf<>::__streambuf_type
#include <string>                       // for basic_string, string

#include <boost/program_options.hpp>    // for value, typed_value, options_description_easy_init, options_description, program_options

namespace po = boost::program_options;

class ProgramOptions {
  public:
    virtual ~ProgramOptions() = default;

    /** Returns generic configuration options - I/O and help. */
    virtual po::options_description getGenericOptions() {
        po::options_description generic("Generic options");
        generic.add_options()
                ("help,h", "produce help message")
                ("cfg,c", po::value<std::string>()->default_value("tests/default.cfg"),
                        "config file path")
                ("policy,p", po::value<std::string>()->default_value("pol.pol"),
                        "policy file path (output)")
                ("seed,s", po::value<unsigned long>()->default_value(0),
                        "RNG seed; use a value of 0 to seed using the current time");
        return generic;
    }

    /** Returns configuration options for the simulation. */
    virtual po::options_description getSimulationOptions() {
        po::options_description simulation("Simulation-specific settings");
        simulation.add_options()
                ("log,l", po::value<std::string>()->default_value("log.log"),
                        "file to log changes to")
                ("changes.hasChanges,u", po::bool_switch(),
                        "whether the PODMP model will change at runtime.")
                ("changes.changesPath,v", po::value<std::string>(),
                        "path to the file with runtime changes to the POMDP model")
                ("simulation.nSteps,n", po::value<long>(),
                        "maximum number of steps to simulate")
                ("simulation.nRuns", po::value<long>(),
                        "number of times to run the simulation")
                ("simulation.savePolicy,z", po::bool_switch(),
                        "whether to save the policy to a fiel after simulation"
                        "completes (this policy can be very large).")
                ("simulation.color", po::value<bool>(),
                        "whether to use color output");
        return simulation;
    }

    /** Returns configuration options for the ABT */
    virtual po::options_description getABTOptions() {
        po::options_description sbt("ABT settings");
        sbt.add_options()
                ("ABT.nParticles", po::value<unsigned long>(),
                        "default number of particles per belief - this number"
                        " will be generated if particle depletion occurs.")
                ("ABT.historiesPerStep", po::value<long>(),
                        "the number of episodes to sample for each step.")
                ("ABT.maximumDepth", po::value<double>(),
                        "maximum Depth allowed before search stops.")
                ("ABT.heuristicExploreCoefficient", po::value<double>(),
                        "determines the ratio between exploration and"
                        " exploitation for the UCB action selection - higher"
                        " values mean more exploration.")
                ("ABT.ucbExploreCoefficient", po::value<double>(),
                        "determines the ratio between exploration and"
                        " exploitation for the UCB action selection - higher"
                        " values mean more exploration.")
                ("ABT.maxNnComparisons", po::value<long>(),
                        "maximum number of nearest-neighbor comparisons to"
                        " make before stopping.")
                ("ABT.maxNnDistance", po::value<double>(),
                        "Maximum distance for a node to be considered a valid"
                        "nearest-neighbor")
                ("ABT.maxObservationDistance", po::value<double>(),
                        "Maximum distance between observations to group them"
                        "together - only applicable if approximate"
                        "observations are being used");
        return sbt;
    }

    /** Returns configurations options for the specific problem */
    virtual po::options_description getProblemOptions() {
        po::options_description problem("Problem settings");
        problem.add_options()
                    ("problem.discountFactor", po::value<double>(),
                            "the discount factor for the POMDP");
        return problem;
    }

    /** Returns configuration options for the specific heuristic used for the
     * problem
     */
    virtual po::options_description getHeuristicOptions() {
        po::options_description heuristic("Heuristic settings");
        return heuristic;
    }
};

#endif /* PROGRAMOPTIONS_HPP_ */
