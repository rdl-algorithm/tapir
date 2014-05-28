#ifndef MODELWITHPROGRAMOPTIONS_HPP_
#define MODELWITHPROGRAMOPTIONS_HPP_

#include <memory>
#include <string>

#include <boost/program_options.hpp>    // for variables_map, variable_value, program_option

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/Model.hpp"             // for Model
#include "solver/action-choosers/choosers.hpp"
#include "solver/belief-q-estimators/estimators.hpp"
#include "solver/search/search_interface.hpp"

#include "problems/shared/parsers.hpp"

namespace po = boost::program_options;

class ModelWithProgramOptions: public virtual solver::Model {
public:
    ModelWithProgramOptions(RandomGenerator *randGen, po::variables_map vm) :
                randGen_(randGen),
                discountFactor_(vm["problem.discountFactor"].as<double>()),
                minParticleCount_(vm["simulation.minParticleCount"].as<unsigned long>()),
                historiesPerStep_(vm["ABT.historiesPerStep"].as<long>()),
                maximumDepth_(vm["ABT.maximumDepth"].as<double>()),
                searchParsers_(),
                estimationParsers_(),
                actionChoosingParsers_(),
                selectionStrategyString_(vm["ABT.selectionStrategy"].as<std::string>()),
                rolloutStrategyString_(vm["ABT.rolloutStrategy"].as<std::string>()),
                estimatorString_(vm["ABT.estimator"].as<std::string>()),
                chooserString_(vm["ABT.chooser"].as<std::string>()),
                hasColorOutput_(vm["color"].as<bool>()),
                hasVerboseOutput_(vm["verbose"].as<bool>()),
                heuristicEnabled_(vm["heuristics.enabled"].as<bool>()) {

        registerSearchParser("ucb", std::make_unique<UcbSearchParser>());
        registerSearchParser("exp3", std::make_unique<Exp3Parser>());
        registerSearchParser("nn", std::make_unique<NnRolloutParser>());
        registerSearchParser("default", std::make_unique<DefaultRolloutParser>());
        registerSearchParser("exp3", std::make_unique<Exp3Parser>());

        registerEstimationParser("mean", std::make_unique<AverageEstimateParser>());
        registerEstimationParser("max", std::make_unique<MaxEstimateParser>());
        registerEstimationParser("robust", std::make_unique<RobustEstimateParser>());

        registerActionChoosingParser("max", std::make_unique<MaxChooserParser>());
        registerActionChoosingParser("robust", std::make_unique<RobustChooserParser>());
        registerActionChoosingParser("ucb", std::make_unique<UcbChooserParser>());
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

    virtual void registerSearchParser(std::string name,
            std::unique_ptr<Parser<std::unique_ptr<solver::SearchStrategy>> > parser) {
        searchParsers_.addParser(name, std::move(parser));
    }
    virtual void registerEstimationParser(std::string name,
            std::unique_ptr<Parser<std::unique_ptr<solver::EstimationStrategy>> > parser) {
        estimationParsers_.addParser(name, std::move(parser));
    }

    virtual void registerActionChoosingParser(std::string name,
            std::unique_ptr<Parser<std::unique_ptr<solver::ActionChoosingStrategy>> > parser) {
        actionChoosingParsers_.addParser(name, std::move(parser));
    }
    virtual std::unique_ptr<solver::SearchStrategy> createSearchStrategy(solver::Solver *solver)
            override {
        return searchParsers_.parse(solver, selectionStrategyString_);
    }
    virtual std::unique_ptr<solver::EstimationStrategy> createEstimationStrategy(
            solver::Solver *solver) override {
        return estimationParsers_.parse(solver, estimatorString_);
    }
    virtual std::unique_ptr<solver::ActionChoosingStrategy> createActionChoosingStrategy(
            solver::Solver *solver) override {
        return actionChoosingParsers_.parse(solver, chooserString_);
    }

private:
    RandomGenerator *randGen_;

// Problem parameters.
    double discountFactor_;

// ABT parameters
    unsigned long minParticleCount_;
    long historiesPerStep_;
    long maximumDepth_;

    ParserSet<std::unique_ptr<solver::SearchStrategy>> searchParsers_;
    ParserSet<std::unique_ptr<solver::EstimationStrategy>> estimationParsers_;
    ParserSet<std::unique_ptr<solver::ActionChoosingStrategy>> actionChoosingParsers_;

    std::string selectionStrategyString_;
    std::string rolloutStrategyString_;
    std::string estimatorString_;
    std::string chooserString_;

    bool hasColorOutput_;
    bool hasVerboseOutput_;
    bool heuristicEnabled_;
}
;

#endif /* MODELWITHPROGRAMOPTIONS_HPP_ */
