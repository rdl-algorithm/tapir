#ifndef SHARED_MODELWITHPROGRAMOPTIONS_HPP_
#define SHARED_MODELWITHPROGRAMOPTIONS_HPP_

#include <memory>
#include <string>

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/Model.hpp"             // for Model
#include "solver/abstract-problem/heuristics/Heuristic.hpp"
#include "solver/belief-estimators/estimators.hpp"
#include "solver/search/search_interface.hpp"
#include "solver/changes/DefaultHistoryCorrector.hpp"

#include "problems/shared/parsers.hpp"
#include "problems/shared/SharedOptions.hpp"

namespace shared {
class ModelWithProgramOptions: public solver::Model {
public:
    ModelWithProgramOptions(std::string problemName, RandomGenerator *randGen,
            std::unique_ptr<SharedOptions> options) :
        Model(problemName, randGen, std::move(options)),
        options_(static_cast<SharedOptions const *>(getOptions())),
        generatorParsers_(),
        heuristicParsers_(),
        searchParsers_(),
        estimationParsers_() {

        registerGeneratorParser("ucb", std::make_unique<UcbParser>());
        registerGeneratorParser("rollout", std::make_unique<DefaultRolloutParser>());
        registerGeneratorParser("nn", std::make_unique<NnRolloutParser>());
        registerGeneratorParser("staged", std::make_unique<StagedParser>(&generatorParsers_));

        registerHeuristicParser("default", std::make_unique<DefaultHeuristicParser>(this));
        registerHeuristicParser("zero", std::make_unique<ZeroHeuristicParser>());

        searchParsers_.setDefaultParser(std::make_unique<BasicSearchParser>(
                &generatorParsers_, &heuristicParsers_, options_->searchHeuristic));
        registerSearchParser("exp3", std::make_unique<Exp3Parser>(&searchParsers_));

        registerEstimationParser("mean", std::make_unique<AverageEstimateParser>());
        registerEstimationParser("max", std::make_unique<MaxEstimateParser>());
        registerEstimationParser("robust", std::make_unique<RobustEstimateParser>());
    }

    virtual ~ModelWithProgramOptions() = default;
    _NO_COPY_OR_MOVE(ModelWithProgramOptions);

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
        return searchParsers_.parse(solver, options_->searchStrategy);
    }
    virtual std::unique_ptr<solver::EstimationStrategy> createEstimationStrategy(
            solver::Solver *solver) override {
        return estimationParsers_.parse(solver, options_->estimator);
    }
    virtual std::unique_ptr<solver::HistoryCorrector> createHistoryCorrector(
            solver::Solver *solver) override {
        return std::make_unique<solver::DefaultHistoryCorrector>(solver,
                heuristicParsers_.parse(solver, options_->searchHeuristic));
    }
    virtual solver::Heuristic getHeuristicFunction() final override {
        return heuristicParsers_.parse(nullptr, options_->searchHeuristic);
    }
    virtual double getDefaultHeuristicValue(solver::HistoryEntry const */*entry*/,
            solver::State const */*state*/, solver::HistoricalData const */*data*/) {
        return 0;
    }

private:
    SharedOptions const *options_;

    ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> generatorParsers_;
    ParserSet<solver::Heuristic> heuristicParsers_;
    ParserSet<std::unique_ptr<solver::SearchStrategy>> searchParsers_;
    ParserSet<std::unique_ptr<solver::EstimationStrategy>> estimationParsers_;
};
} /* namespace shared */

#endif /* SHARED_MODELWITHPROGRAMOPTIONS_HPP_ */
