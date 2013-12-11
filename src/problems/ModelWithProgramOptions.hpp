#ifndef MODELWITHPROGRAMOPTIONS_HPP
#define MODELWITHPROGRAMOPTIONS_HPP

#include <boost/program_options.hpp>    // for variables_map, variable_value, program_options

#include "defs.hpp"                     // for RandomGenerator
#include "solver/Model.hpp"             // for Model

namespace po = boost::program_options;

class ModelWithProgramOptions : public Model {
  public:
    ModelWithProgramOptions(RandomGenerator *randGen, po::variables_map vm) :
        Model(randGen),
        discount(vm["problem.discount"].as<double>()),
        nParticles(vm["SBT.nParticles"].as<long>()),
        maxTrials(vm["SBT.maxTrials"].as<long>()),
        depthTh(vm["SBT.depthTh"].as<double>()),
        coefUCB(vm["SBT.coefUCB"].as<double>()),
        exploreCoef(vm["SBT.exploreCoef"].as<double>()),
        maxDistTry(vm["SBT.maxDistTry"].as<long>()),
        distTh(vm["SBT.distTh"].as<double>()) {
    }

    virtual ~ModelWithProgramOptions() = default;

    // Simple getters
    virtual double getDiscountFactor() final {
        return discount;
    }

    virtual unsigned long getNParticles() final {
        return nParticles;
    }
    virtual unsigned long getMaxTrials() final {
        return maxTrials;
    }
    virtual double getDepthTh() final {
        return depthTh;
    }
    virtual double getCoefUCB() final {
        return coefUCB;
    }
    virtual double getExploreCoef() final {
        return exploreCoef;
    }
    virtual long getMaxDistTry() final {
        return maxDistTry;
    }
    virtual double getDistTh() final {
        return distTh;
    }

  private:
    // Problem parameters.
    double discount;

    // SBT parameters
    unsigned long nParticles;
    unsigned long maxTrials;
    double depthTh;
    double coefUCB;
    double exploreCoef;

    long maxDistTry;
    double distTh;
};

#endif /* MODELWITHPROGRAMOPTIONS_HPP */
