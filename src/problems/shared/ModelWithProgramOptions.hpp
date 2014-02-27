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
        maximumDepth_(vm["SBT.maximumDepth"].as<double>()),
        heuristicExploreCoefficient_(
                vm["SBT.heuristicExploreCoefficient"].as<double>()),
        ucbExploreCoefficient_(vm["SBT.ucbExploreCoefficient"].as<double>()),
        maxNnComparisons_(vm["SBT.maxNnComparisons"].as<long>()),
        maxNnDistance_(vm["SBT.maxNnDistance"].as<double>())
    {
    }

    virtual ~ModelWithProgramOptions() = default;
    _NO_COPY_OR_MOVE(ModelWithProgramOptions);

    // Simple getters
    virtual RandomGenerator *getRandomGenerator() override {
        return randGen_;
    }

    virtual double getDiscountFactor() override {
        return discountFactor_;
    }
    virtual long getNParticles() override {
        return nParticles_;
    }
    virtual long getMaxTrials() override {
        return maxTrials_;
    }
    virtual long getMaximumDepth() override {
        return maximumDepth_;
    }
    virtual double getUcbExploreCoefficient() override {
        return ucbExploreCoefficient_;
    }
    virtual double getHeuristicExploreCoefficient() override {
        return heuristicExploreCoefficient_;
    }
    virtual long getMaxNnComparisons() override {
        return maxNnComparisons_;
    }
    virtual double getMaxNnDistance() override {
        return maxNnDistance_;
    }

  private:
    RandomGenerator *randGen_;

    // Problem parameters.
    double discountFactor_;

    // SBT parameters
    long nParticles_;
    long maxTrials_;
    long maximumDepth_;

    double heuristicExploreCoefficient_;
    double ucbExploreCoefficient_;

    long maxNnComparisons_;
    double maxNnDistance_;
};

#endif /* MODELWITHPROGRAMOPTIONS_HPP_ */
