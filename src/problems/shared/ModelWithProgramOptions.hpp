#ifndef MODELWITHPROGRAMOPTIONS_HPP_
#define MODELWITHPROGRAMOPTIONS_HPP_

#include <memory>
#include <string>

#include <boost/program_options.hpp>    // for variables_map, variable_value, program_option

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/Model.hpp"             // for Model
#include "solver/abstract-problem/heuristics/Heuristic.hpp"
#include "solver/belief-estimators/estimators.hpp"
#include "solver/search/search_interface.hpp"
#include "solver/changes/DefaultHistoryCorrector.hpp"

#include "problems/shared/parsers.hpp"

namespace po = boost::program_options;

class ModelWithProgramOptions: public solver::Model {
public:
    ModelWithProgramOptions(RandomGenerator *randGen, po::variables_map vm) :
                randGen_(randGen),
                discountFactor_(vm["problem.discountFactor"].as<double>()),
                minParticleCount_(vm["simulation.minParticleCount"].as<unsigned long>()),
                historiesPerStep_(vm["ABT.historiesPerStep"].as<long>()),
                stepTimeout_(vm["ABT.stepTimeout"].as<double>()),
                maximumDepth_(vm["ABT.maximumDepth"].as<long>()),
                generatorParsers_(),
                heuristicParsers_(),
                searchParsers_(),
                estimationParsers_(),
                heuristicString_(vm["ABT.searchHeuristic"].as<std::string>()),
                searchStrategyString_(vm["ABT.searchStrategy"].as<std::string>()),
                estimatorString_(vm["ABT.estimator"].as<std::string>()),
                hasColorOutput_(vm["color"].as<bool>()),
                hasVerboseOutput_(vm["verbose"].as<bool>()) {

        registerGeneratorParser("ucb", std::make_unique<UcbParser>());
        registerGeneratorParser("rollout", std::make_unique<DefaultRolloutParser>());
        registerGeneratorParser("nn", std::make_unique<NnRolloutParser>());
        registerGeneratorParser("staged", std::make_unique<StagedParser>(&generatorParsers_));

        registerHeuristicParser("default", std::make_unique<DefaultHeuristicParser>(this));
        registerHeuristicParser("zero", std::make_unique<ZeroHeuristicParser>());

        searchParsers_.setDefaultParser(std::make_unique<BasicSearchParser>(
                &generatorParsers_, &heuristicParsers_, heuristicString_));
        registerSearchParser("exp3", std::make_unique<Exp3Parser>(&searchParsers_));

        registerEstimationParser("mean", std::make_unique<AverageEstimateParser>());
        registerEstimationParser("max", std::make_unique<MaxEstimateParser>());
        registerEstimationParser("robust", std::make_unique<RobustEstimateParser>());
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
    virtual unsigned long getMinParticleCount() override {
        return minParticleCount_;
    }
    virtual long getNumberOfHistoriesPerStep() override {
        return historiesPerStep_;
    }
    virtual double getStepTimeout() override {
        return stepTimeout_;
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

    virtual void registerGeneratorParser(std::string name,
            std::unique_ptr<Parser<std::unique_ptr<solver::StepGeneratorFactory>> > parser) {
        generatorParsers_.addParser(name, std::move(parser));
    }
    virtual void registerHeuristicParser(std::string name,
            std::unique_ptr<Parser<solver::Heuristic> > parser) {
        heuristicParsers_.addParser(name, std::move(parser));
    }
    virtual void registerSearchParser(std::string name,
            std::unique_ptr<Parser<std::unique_ptr<solver::SearchStrategy>> > parser) {
        searchParsers_.addParser(name, std::move(parser));
    }
    virtual void registerEstimationParser(std::string name,
            std::unique_ptr<Parser<std::unique_ptr<solver::EstimationStrategy>> > parser) {
        estimationParsers_.addParser(name, std::move(parser));
    }

    virtual std::unique_ptr<solver::SearchStrategy> createSearchStrategy(solver::Solver *solver)
            override {
        return searchParsers_.parse(solver, searchStrategyString_);
    }
    virtual std::unique_ptr<solver::EstimationStrategy> createEstimationStrategy(
            solver::Solver *solver) override {
        return estimationParsers_.parse(solver, estimatorString_);
    }
    virtual std::unique_ptr<solver::HistoryCorrector> createHistoryCorrector(
            solver::Solver *solver) override {
        return std::make_unique<solver::DefaultHistoryCorrector>(solver,
                heuristicParsers_.parse(solver, heuristicString_));
    }
    virtual solver::Heuristic getHeuristicFunction() final override {
        return heuristicParsers_.parse(nullptr, heuristicString_);
    }
    virtual double getDefaultHeuristicValue(solver::HistoryEntry const */*entry*/,
            solver::State const */*state*/, solver::HistoricalData const */*data*/) {
        return 0;
    }

private:
    RandomGenerator *randGen_;

// Problem parameters.
    double discountFactor_;

// ABT parameters
    unsigned long minParticleCount_;
    long historiesPerStep_;
    double stepTimeout_;
    long maximumDepth_;

    ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> generatorParsers_;
    ParserSet<solver::Heuristic> heuristicParsers_;
    ParserSet<std::unique_ptr<solver::SearchStrategy>> searchParsers_;
    ParserSet<std::unique_ptr<solver::EstimationStrategy>> estimationParsers_;

    std::string heuristicString_;
    std::string searchStrategyString_;
    std::string estimatorString_;

    bool hasColorOutput_;
    bool hasVerboseOutput_;
};

#endif /* MODELWITHPROGRAMOPTIONS_HPP_ */
