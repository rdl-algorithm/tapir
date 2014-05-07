#ifndef MODELWITHPROGRAMOPTIONS_HPP_
#define MODELWITHPROGRAMOPTIONS_HPP_

#include <boost/program_options.hpp>    // for variables_map, variable_value, program_option

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/Model.hpp"             // for Model

#include "parsers.hpp"

namespace po = boost::program_options;

class ModelWithProgramOptions: public virtual solver::Model {
public:
    ModelWithProgramOptions(RandomGenerator *randGen, po::variables_map vm) :
                randGen_(randGen),
                discountFactor_(vm["problem.discountFactor"].as<double>()),
                minParticleCount_(vm["simulation.minParticleCount"].as<unsigned long>()),
                historiesPerStep_(vm["ABT.historiesPerStep"].as<long>()),
                maximumDepth_(vm["ABT.maximumDepth"].as<double>()),
                selectionParsers_(),
                rolloutParsers_(),
                backpropParsers_(),
                selectionStrategyString_(vm["ABT.selectionStrategy"].as<std::string>()),
                rolloutStrategyString_(vm["ABT.rolloutStrategy"].as<std::string>()),
                backpropagationStrategyString_(vm["ABT.backpropagationStrategy"].as<std::string>()),
                hasColorOutput_(vm["color"].as<bool>()),
                hasVerboseOutput_(vm["verbose"].as<bool>()),
                heuristicEnabled_(vm["heuristics.enabled"].as<bool>()) {
        registerSelectionParser("ucb", std::make_unique<UcbSearchParser>());
        registerSelectionParser("exp3", std::make_unique<Exp3Parser>());

        registerRolloutParser("nn", std::make_unique<NnRolloutParser>());
        registerRolloutParser("default", std::make_unique<DefaultRolloutParser>());
        registerRolloutParser("exp3", std::make_unique<Exp3Parser>());

        registerBackpropagationParser("mean", std::make_unique<AveragePropagatorParser>());
        registerBackpropagationParser("max", std::make_unique<MaximumPropagatorParser>());
        registerBackpropagationParser("robust", std::make_unique<RobustPropagatorParser>());
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
    virtual void registerSelectionParser(std::string name,
            std::unique_ptr<Parser<solver::SearchStrategy>> parser) {
        selectionParsers_.addParser(name, std::move(parser));
    }
    virtual void registerRolloutParser(std::string name,
            std::unique_ptr<Parser<solver::SearchStrategy>> parser) {
        rolloutParsers_.addParser(name, std::move(parser));
    }
    virtual void registerBackpropagationParser(std::string name,
            std::unique_ptr<Parser<solver::BackpropagationStrategy>> parser) {
        backpropParsers_.addParser(name, std::move(parser));
    }
    virtual std::unique_ptr<solver::SearchStrategy> createSelectionStrategy(
            solver::Solver *solver) override {
        return selectionParsers_.parse(solver, selectionStrategyString_);
    }
    virtual std::unique_ptr<solver::SearchStrategy> createRolloutStrategy(
            solver::Solver *solver) override {
        return rolloutParsers_.parse(solver, rolloutStrategyString_);
    }
    virtual std::unique_ptr<solver::BackpropagationStrategy> createBackpropagationStrategy(
            solver::Solver *solver) override {
        return backpropParsers_.parse(solver, backpropagationStrategyString_);
    }

private:
    RandomGenerator *randGen_;

// Problem parameters.
    double discountFactor_;

// ABT parameters
    unsigned long minParticleCount_;
    long historiesPerStep_;
    long maximumDepth_;

    ParserSet<solver::SearchStrategy> selectionParsers_;
    ParserSet<solver::SearchStrategy> rolloutParsers_;
    ParserSet<solver::BackpropagationStrategy> backpropParsers_;

    std::string selectionStrategyString_;
    std::string rolloutStrategyString_;
    std::string backpropagationStrategyString_;

    bool hasColorOutput_;
    bool hasVerboseOutput_;
    bool heuristicEnabled_;
};

#endif /* MODELWITHPROGRAMOPTIONS_HPP_ */
