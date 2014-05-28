#ifndef PROGRAMOPTIONS_HPP_
#define PROGRAMOPTIONS_HPP_

#include <sstream>                      // for basic_stringbuf<>::int_type, basic_stringbuf<>::pos_type, basic_stringbuf<>::__streambuf_type
#include <string>                       // for basic_string, string

#include <boost/program_options.hpp>    // for value, typed_value, options_description_easy_init, options_description, program_options

namespace po = boost::program_options;

void load_overrides(po::variables_map &vm) {
    if (!vm["sel"].empty()) {
        vm.at("ABT.selectionStrategy") = vm["sel"];
    }
    if (!vm["rol"].empty()) {
        vm.at("ABT.rolloutStrategy") = vm["rol"];
    }
    if (!vm["est"].empty()) {
        vm.at("ABT.estimator") = vm["est"];
    }
    if (!vm["cho"].empty()) {
        vm.at("ABT.estimator") = vm["cho"];
    }
}

class ProgramOptions {
  public:
    virtual ~ProgramOptions() = default;

    /** Returns generic configuration options - I/O and help. */
    virtual po::options_description getGenericOptions() {
        po::options_description generic("Generic options");
        generic.add_options()
                ("help", "produce help message")
                ("cfg,f", po::value<std::string>()->default_value("tests/default.cfg"),
                        "config file path")
                ("policy,p", po::value<std::string>()->default_value("pol.pol"),
                        "policy file path (output)")
                ("seed,s", po::value<unsigned long>()->default_value(0),
                        "RNG seed; use a value of 0 to seed using the current time")
                ("color,c", po::value<bool>()->default_value(false)->implicit_value(true),
                        "whether to use color output")
                ("verbose,v", po::value<bool>()->default_value(false)->implicit_value(true),
                        "whether to use verbose output");
        return generic;
    }

    /** Returns configuration options for the simulation. */
    virtual po::options_description getSimulationOptions() {
        po::options_description simulation("Simulation-specific settings");
        simulation.add_options()
                ("log,l", po::value<std::string>()->default_value("log.log"),
                        "file to log changes to")
                ("changes.hasChanges,u", po::value<bool>()->default_value(false)->implicit_value(true),
                        "whether the PODMP model will change at runtime.")
                ("changes.changesPath,g", po::value<std::string>(),
                        "path to the file with runtime changes to the POMDP model")
                ("simulation.nSteps,n", po::value<long>(),
                        "maximum number of steps to simulate")
                ("simulation.minParticleCount", po::value<unsigned long>(),
                        "Minimum number of particles per belief - if the "
                        "particle count drops below this number additional "
                        "particles will be resampled.")
                ("simulation.nRuns,r", po::value<long>(),
                        "number of times to run the simulation")
                ("simulation.savePolicy,a", po::value<bool>()->default_value(false)->implicit_value(true),
                        "whether to save the policy to a file after simulation"
                        " completes (this policy can be very large).");
        return simulation;
    }

    /** Returns configuration options for the ABT */
    virtual po::options_description getABTOptions() {
        po::options_description abt("ABT settings");
        abt.add_options()
                ("sel", po::value<std::string>(),
                        "overriding alias for ABT.selectionStrategy")
                ("rol", po::value<std::string>(),
                        "overriding alias for ABT.rolloutStrategy")
                ("est", po::value<std::string>(),
                        "overriding alias for ABT.estimator")
                ("cho", po::value<std::string>(),
                        "overriding alias for ABT.chooser")
                ("ABT.historiesPerStep,i", po::value<long>(),
                        "the number of episodes to sample for each step.")
                ("ABT.maximumDepth", po::value<double>(),
                        "maximum Depth allowed before search stops.")
                ("ABT.selectionStrategy", po::value<std::string>(),
                        "the search strategy to use")
                ("ABT.rolloutStrategy", po::value<std::string>(),
                        "the rollout strategy to use")
                ("ABT.estimator", po::value<std::string>(),
                        "the function that estimates the q-value of a belief.")
                ("ABT.chooser", po::value<std::string>(),
                        "the function that chooses the action to take.")
                ("ABT.maxObservationDistance", po::value<double>(),
                        "Maximum distance between observations to group them"
                        " together - only applicable if approximate"
                        " observations are being used");
        return abt;
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
        po::options_description heuristics("Heuristic settings");
        heuristics.add_options()
                ("heuristics.enabled,h", po::value<bool>()->default_value(true)->implicit_value(true),
                        "whether any heuristic should be used - no heuristic"
                        " means a default value will always be returned.");
        return heuristics;
    }
};

#endif /* PROGRAMOPTIONS_HPP_ */
