/** @file ModelWithProgramOptions.hpp
 *
 * Offers a Model class with extra functionality, allowing a text-based interface for selecting
 * different search strategies.
 */
#ifndef SHARED_MODELWITHPROGRAMOPTIONS_HPP_
#define SHARED_MODELWITHPROGRAMOPTIONS_HPP_

#include <memory>
#include <string>

#include "global.hpp"                     // for RandomGenerator

#include "solver/abstract-problem/Model.hpp"             // for Model
#include "solver/abstract-problem/heuristics/HeuristicFunction.hpp"
#include "solver/belief-estimators/estimators.hpp"
#include "solver/search/search_interface.hpp"
#include "solver/changes/DefaultHistoryCorrector.hpp"

#include "problems/shared/parsers.hpp"
#include "problems/shared/SharedOptions.hpp"

/** A namespace for useful functionality that is shared by various different problems. */
namespace shared {
/** A partial implementation of Model allowing for text-based parsing of various advanced search
 * options.
 *
 * In particular, these three configuration settings are made available in the configuration file
 * parser, with several options that are available by default:
 * - SharedOptions::searchHeuristic - the heuristic function to use, e.g. "zero()"
 * - SharedOptions::searchStrategy - the search strategy to use, e.g. "ucb(15.0)"
 * - SharedOptions::estimator - the belief value estimator to use, e.g. "mean()"
 */
class ModelWithProgramOptions: public solver::Model {
public:
	/** Creates a new ModelWithProgramOptions with the given problem name, the given
	 * RandomGenerator, and the given set of options.
	 */
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

    /** Associates the given parser for instances of StepGeneratorFactory with the given name
     * string, allowing it to be parsed at runtime.
     *
     * The created instance of StepGeneratorFactory will then serve to create StepGenerator
     * instances that will generate search steps one by one.
     *
     * See ucb_search.hpp for a canonical example of one of these generators; the UCB strategy
     * is then parsed by the UcbParser class.
     */
    virtual void registerGeneratorParser(std::string name,
            std::unique_ptr<Parser<std::unique_ptr<solver::StepGeneratorFactory>> > parser) {
        generatorParsers_.addParser(name, std::move(parser));
    }

    /** Associates the given parser for heuristic functions with the given name string, allowing
     * it to be parsed at runtime.
     *
     * The parsed heuristic can then be applied to any sequence that finishes without reaching
     * a terminal state - the reward value for the last entry in the sequence will contain the
     * heuristic value. In particular, SearchStrategy or HistoryCorrector implementations will
     * typically use this heuristic when they end a sequence early; that heuristic value will then
     * be propagated back up the policy tree.
     */
    virtual void registerHeuristicParser(std::string name,
            std::unique_ptr<Parser<solver::HeuristicFunction> > parser) {
        heuristicParsers_.addParser(name, std::move(parser));
    }

    /** Associates the given parser for search strategies with the given name string, allowing it
     * to be parsed at runtime.
     *
     * Most typical search strategies should be implemented by wrapping a StepGeneratorFactory
     * in an instance of BasicSearchStrategy. However, if the search strategy needs to perform
     * extra functionality it may be necessary to make a separate instance.
     *
     * The EXP3-based combination of strategies (see MultipleStrategiesExp3.hpp) is one example of
     * this - it wraps multiple strategies into one, and needs to update the statistics for the
     * EXP3 algorithm after it finishes a search.
     */
    virtual void registerSearchParser(std::string name,
            std::unique_ptr<Parser<std::unique_ptr<solver::SearchStrategy>> > parser) {
        searchParsers_.addParser(name, std::move(parser));
    }
    virtual void registerEstimationParser(std::string name,
            std::unique_ptr<Parser<std::unique_ptr<solver::EstimationStrategy>> > parser) {
        estimationParsers_.addParser(name, std::move(parser));
    }


    // Overridden methods follow
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
    virtual solver::HeuristicFunction getHeuristicFunction() final override {
        return heuristicParsers_.parse(nullptr, options_->searchHeuristic);
    }
    virtual double getDefaultHeuristicValue(solver::HistoryEntry const */*entry*/,
            solver::State const */*state*/, solver::HistoricalData const */*data*/) {
        return 0;
    }

private:
    SharedOptions const *options_;

    ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> generatorParsers_;
    ParserSet<solver::HeuristicFunction> heuristicParsers_;
    ParserSet<std::unique_ptr<solver::SearchStrategy>> searchParsers_;
    ParserSet<std::unique_ptr<solver::EstimationStrategy>> estimationParsers_;
};
} /* namespace shared */

#endif /* SHARED_MODELWITHPROGRAMOPTIONS_HPP_ */
