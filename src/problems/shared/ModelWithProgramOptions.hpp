#ifndef MODELWITHPROGRAMOPTIONS_HPP_
#define MODELWITHPROGRAMOPTIONS_HPP_

#include <boost/program_options.hpp>    // for variables_map, variable_value, program_options
#include <boost/regex.hpp>    // for variables_map, variable_value, program_options

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/Model.hpp"             // for Model

#include "solver/search/UcbSearchStrategy.hpp"
#include "solver/search/RandomRolloutStrategy.hpp"
#include "solver/search/MultipleStrategiesExp3.hpp"
#include "solver/search/NnRolloutStrategy.hpp"

namespace po = boost::program_options;

class ModelWithProgramOptions: public virtual solver::Model {
public:
    ModelWithProgramOptions(RandomGenerator *randGen, po::variables_map vm) :
                randGen_(randGen),
                discountFactor_(vm["problem.discountFactor"].as<double>()),
                nParticles_(vm["ABT.nParticles"].as<unsigned long>()),
                historiesPerStep_(vm["ABT.historiesPerStep"].as<long>()),
                maximumDepth_(vm["ABT.maximumDepth"].as<double>()),
                searchStrategyString_(vm["ABT.searchStrategy"].as<std::string>()),
                rolloutStrategyString_(vm["ABT.rolloutStrategy"].as<std::string>()),
                hasColorOutput_(vm["color"].as<bool>()),
                hasVerboseOutput_(vm["verbose"].as<bool>()),
                heuristicEnabled_(vm["heuristic.enabled"].as<bool>()) {
    }

    virtual ~ModelWithProgramOptions() = default;
    _NO_COPY_OR_MOVE(ModelWithProgramOptions)
    ;

// Simple getters
    virtual RandomGenerator *getRandomGenerator() override {
        return randGen_;
    }

    virtual double getDiscountFactor() override {
        return discountFactor_;
    }
    virtual unsigned long getNParticles() override {
        return nParticles_;
    }
    virtual long getNumberOfHistoriesPerStep() override {
        return historiesPerStep_;
    }
    virtual long getMaximumDepth() override {
        return maximumDepth_;
    }
    virtual bool hasColorOutput() override {
        return hasColorOutput_;
    }
    virtual bool hasVerboseOutput() override {
        return hasVerboseOutput_;
    }
    virtual bool heuristicEnabled() {
        return heuristicEnabled_;
    }
    virtual std::unique_ptr<solver::SearchStrategy> parseStrategy(
            solver::Solver *solver, std::string strategyString) {
        boost::regex pattern(" *(.+?)\\((.*)\\) *");
        boost::smatch results;
        if (!boost::regex_match(strategyString, results, pattern)) {
            return nullptr;
        }
        std::string strategyType = results[1];
        std::string argsString = results[2];

        std::vector<std::string> args;
        std::string::iterator prevIter = argsString.begin();
        int parenCount = 0;
        for (std::string::iterator charIter = argsString.begin();
                charIter != argsString.end(); charIter++) {
            if (*charIter == '(') {
                parenCount++;
                continue;
            }
            if (*charIter == ')') {
                parenCount--;
                continue;
            }
            if (*charIter == ',' && parenCount == 0) {
                if (prevIter != charIter) {
                    args.push_back(std::string(prevIter, charIter));
                }
                prevIter = charIter + 1;
            }
        }
        if (prevIter != argsString.end()) {
            args.push_back(std::string(prevIter, argsString.end()));
        }
        if (strategyType == "ucb") {
            double explorationCoefficient;
            std::istringstream(args[0]) >> explorationCoefficient;
            return std::make_unique<solver::UcbSearchStrategy>(solver,
                    explorationCoefficient);
        }
        if (strategyType == "nn") {
            long maxNnComparisons;
            double maxNnDistance;
            std::istringstream(args[0]) >> maxNnComparisons;
            std::istringstream(args[1]) >> maxNnDistance;
            return std::make_unique<solver::NnRolloutStrategy>(solver,
                    maxNnComparisons, maxNnDistance);
        }
        if (strategyType == "random") {
            long maxNSteps;
            std::istringstream(args[0]) >> maxNSteps;
            return std::make_unique<solver::RandomRolloutStrategy>(solver,
                    maxNSteps);
        }
        if (strategyType == "exp3") {
            std::vector<std::unique_ptr<solver::SearchStrategy>> strategies;
            std::vector<std::string>::iterator it = args.begin();
            double strategyExplorationCoefficient;
            std::istringstream(*it) >> strategyExplorationCoefficient;
            it++;
            for (; it != args.end(); it++) {
                strategies.push_back(parseStrategy(solver, *it));
            }
            return std::make_unique<solver::MultipleStrategiesExp3>(solver,
                    strategyExplorationCoefficient, std::move(strategies));
        }
        return nullptr;
    }
    virtual std::unique_ptr<solver::SearchStrategy> createSearchStrategy(
            solver::Solver *solver) override {
        return parseStrategy(solver, searchStrategyString_);
    }
    virtual std::unique_ptr<solver::SearchStrategy> createRolloutStrategy(
            solver::Solver *solver) override {
        return parseStrategy(solver, rolloutStrategyString_);
    }

private:
    RandomGenerator *randGen_;

// Problem parameters.
    double discountFactor_;

// ABT parameters
    unsigned long nParticles_;
    long historiesPerStep_;
    long maximumDepth_;

    std::string searchStrategyString_;
    std::string rolloutStrategyString_;

    bool hasColorOutput_;
    bool hasVerboseOutput_;
    bool heuristicEnabled_;
};

#endif /* MODELWITHPROGRAMOPTIONS_HPP_ */
