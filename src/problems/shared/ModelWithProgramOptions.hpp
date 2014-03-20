#ifndef MODELWITHPROGRAMOPTIONS_HPP_
#define MODELWITHPROGRAMOPTIONS_HPP_

#include <boost/program_options.hpp>    // for variables_map, variable_value, program_option

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/Model.hpp"             // for Model

#include "solver/search/UcbSearchStrategy.hpp"
#include "solver/search/RandomRolloutStrategy.hpp"
#include "solver/search/MultipleStrategiesExp3.hpp"
#include "solver/search/NnRolloutStrategy.hpp"

#include "strategy_parsers.hpp"

namespace po = boost::program_options;

class ModelWithProgramOptions: public virtual solver::Model {
public:
    ModelWithProgramOptions(RandomGenerator *randGen, po::variables_map vm) :
                randGen_(randGen),
                discountFactor_(vm["problem.discountFactor"].as<double>()),
                nParticles_(vm["ABT.nParticles"].as<unsigned long>()),
                historiesPerStep_(vm["ABT.historiesPerStep"].as<long>()),
                maximumDepth_(vm["ABT.maximumDepth"].as<double>()),
                allParser_(),
                searchStrategyString_(vm["ABT.searchStrategy"].as<std::string>()),
                rolloutStrategyString_(vm["ABT.rolloutStrategy"].as<std::string>()),
                hasColorOutput_(vm["color"].as<bool>()),
                hasVerboseOutput_(vm["verbose"].as<bool>()),
                heuristicEnabled_(vm["heuristic.enabled"].as<bool>()) {
        registerParser("ucb", std::make_unique<UcbSearchParser>());
        registerParser("nn", std::make_unique<NnRolloutParser>());
        registerParser("random", std::make_unique<RandomRolloutParser>());
        registerParser("exp3", std::make_unique<Exp3Parser>());
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
    virtual void registerParser(std::string name,
            std::unique_ptr<StrategyParser> parser) {
        allParser_.addParser(name, std::move(parser));
    }
    virtual std::unique_ptr<solver::SearchStrategy> parseStrategy(
            solver::Solver *solver, std::string strategyString) {
        return allParser_.parseStrategy(solver, strategyString);
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

    AllStrategiesParser allParser_;

    std::string searchStrategyString_;
    std::string rolloutStrategyString_;

    bool hasColorOutput_;
    bool hasVerboseOutput_;
    bool heuristicEnabled_;
};

#endif /* MODELWITHPROGRAMOPTIONS_HPP_ */
