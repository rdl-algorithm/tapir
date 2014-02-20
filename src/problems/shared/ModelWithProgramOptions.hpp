#ifndef MODELWITHPROGRAMOPTIONS_HPP_
#define MODELWITHPROGRAMOPTIONS_HPP_

#include <boost/program_options.hpp>    // for variables_map, variable_value, program_options

#include "global.hpp"                     // for RandomGenerator
#include "solver/Model.hpp"             // for Model

namespace po = boost::program_options;

class ModelWithProgramOptions : public virtual solver::Model {
  public:
    ModelWithProgramOptions(RandomGenerator *randGen, po::variables_map vm) :
        randGen_(randGen),
        discountFactor_(vm["problem.discountFactor"].as<double>()),
        nParticles_(vm["SBT.nParticles"].as<long>()),
        maxTrials_(vm["SBT.maxTrials"].as<long>()),
        minimumDiscount_(vm["SBT.minimumDiscount"].as<double>()),
        heuristicExploreCoefficient_(
                vm["SBT.heuristicExploreCoefficient"].as<double>()),
        ucbExploreCoefficient_(vm["SBT.ucbExploreCoefficient"].as<double>()),
        maxNnComparisons_(vm["SBT.maxNnComparisons"].as<long>()),
        maxNnDistance_(vm["SBT.maxNnDistance"].as<double>()) {
    }

    virtual ~ModelWithProgramOptions() = default;
    _NO_COPY_OR_MOVE(ModelWithProgramOptions);

    // Simple getters
    virtual RandomGenerator *getRandomGenerator() {
        return randGen_;
    }

    virtual double getDiscountFactor() const {
        return discountFactor_;
    }
    virtual long getNParticles() const {
        return nParticles_;
    }
    virtual long getMaxTrials() const {
        return maxTrials_;
    }
    virtual double getMinimumDiscount() const {
        return minimumDiscount_;
    }
    virtual double getUcbExploreCoefficient() const {
        return ucbExploreCoefficient_;
    }
    virtual double getHeuristicExploreCoefficient() const {
        return heuristicExploreCoefficient_;
    }
    virtual long getMaxNnComparisons() const {
        return maxNnComparisons_;
    }
    virtual double getMaxNnDistance() const {
        return maxNnDistance_;
    }

  private:
    RandomGenerator *randGen_;

    // Problem parameters.
    double discountFactor_;

    // SBT parameters
    long nParticles_;
    long maxTrials_;
    double minimumDiscount_;

    double heuristicExploreCoefficient_;
    double ucbExploreCoefficient_;

    long maxNnComparisons_;
    double maxNnDistance_;
};

#endif /* MODELWITHPROGRAMOPTIONS_HPP_ */
