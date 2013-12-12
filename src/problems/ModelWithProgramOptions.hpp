#ifndef MODELWITHPROGRAMOPTIONS_HPP_
#define MODELWITHPROGRAMOPTIONS_HPP_

#include <boost/program_options.hpp>    // for variables_map, variable_value, program_options

#include "defs.hpp"                     // for RandomGenerator
#include "solver/Model.hpp"             // for Model

namespace po = boost::program_options;

class ModelWithProgramOptions : public solver::Model {
  public:
    ModelWithProgramOptions(RandomGenerator *randGen, po::variables_map vm) :
        solver::Model(randGen),
        discountFactor_(vm["problem.discountFactor"].as<double>()),
        nParticles_(vm["SBT.nParticles"].as<unsigned long>()),
        maxTrials_(vm["SBT.maxTrials"].as<unsigned long>()),
        minimumDiscount_(vm["SBT.minimumDiscount"].as<double>()),
        heuristicExploreCoefficient_(
                vm["SBT.heuristicExploreCoefficient"].as<double>()),
        ucbExploreCoefficient_(vm["SBT.ucbExploreCoefficient"].as<double>()),
        maxNnComparisons_(vm["SBT.maxNnComparisons"].as<long>()),
        maxNnDistance_(vm["SBT.maxNnDistance"].as<double>()) {
    }

    virtual ~ModelWithProgramOptions() = default;

    // Simple getters
    virtual double getDiscountFactor() final {
        return discountFactor_;
    }
    virtual unsigned long getNParticles() final {
        return nParticles_;
    }
    virtual unsigned long getMaxTrials() final {
        return maxTrials_;
    }
    virtual double getMinimumDiscount() final {
        return minimumDiscount_;
    }
    virtual double getUcbExploreCoefficient() final {
        return ucbExploreCoefficient_;
    }
    virtual double getHeuristicExploreCoefficient() final {
        return heuristicExploreCoefficient_;
    }
    virtual long getMaxNnComparisons() final {
        return maxNnComparisons_;
    }
    virtual double getMaxNnDistance() final {
        return maxNnDistance_;
    }

  private:
    // Problem parameters.
    double discountFactor_;

    // SBT parameters
    unsigned long nParticles_;
    unsigned long maxTrials_;
    double minimumDiscount_;

    double heuristicExploreCoefficient_;
    double ucbExploreCoefficient_;

    long maxNnComparisons_;
    double maxNnDistance_;
};

#endif /* MODELWITHPROGRAMOPTIONS_HPP_ */
