/** @file parsers.cpp
 *
 * Provides implementations for the parsing of various ABT parameters, allowing for additional
 * customization at runtime instead of compile-time.
 */
#include "parsers.hpp"

#include <iostream>

#include "global.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/Solver.hpp"

#include "solver/abstract-problem/heuristics/HeuristicFunction.hpp"

#include "solver/belief-estimators/estimators.hpp"

#include "solver/search/search_interface.hpp"
#include "solver/search/MultipleStrategiesExp3.hpp"
#include "solver/search/steppers/ucb_search.hpp"
#include "solver/search/steppers/gps_search.hpp"
#include "solver/search/steppers/default_rollout.hpp"
#include "solver/search/steppers/nn_rollout.hpp"

#include "problems/shared/ModelWithProgramOptions.hpp"

namespace shared {
std::vector<std::string> split_function(std::string text) {
    std::size_t i0 = text.find('(');
    std::size_t i1 = text.rfind(')');
    if (i0 == std::string::npos || i1 == std::string::npos || i1 <= i0) {
        return std::vector<std::string>( { text });
    }

    std::string function = text.substr(0, i0);
    tapir::trim(function);
    std::vector<std::string> argsVector;
    argsVector.push_back(function);

    std::string argsString = text.substr(i0 + 1, i1 - i0 - 1);
    std::string::iterator prevIter = argsString.begin();
    int parenCount = 0;
    for (std::string::iterator charIter = argsString.begin(); charIter != argsString.end();
            charIter++) {
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
                std::string s(prevIter, charIter);
                tapir::trim(s);
                argsVector.push_back(s);
            }
            prevIter = charIter + 1;
        }
    }
    if (prevIter != argsString.end()) {
        std::string s(prevIter, argsString.end());
        tapir::trim(s);
        argsVector.push_back(s);
    }
    return argsVector;
}

std::unique_ptr<solver::StepGeneratorFactory> UcbParser::parse(solver::Solver *solver,
        std::vector<std::string> args) {
    double explorationCoefficient;
    std::istringstream(args[1]) >> explorationCoefficient;
    return std::make_unique<solver::UcbStepGeneratorFactory>(solver, explorationCoefficient);
}

std::unique_ptr<solver::StepGeneratorFactory> GpsParser::parse(solver::Solver *solver, std::vector<std::string> args) {

	using solver::choosers::GpsChooserOptions;
	GpsChooserOptions options;

	size_t index = 1;

	if (args.size() > index) {
		if (args[index] == "golden") {
			options.searchType = GpsChooserOptions::GOLDEN;
		} else if (args[index] == "compass") {
			options.searchType = GpsChooserOptions::COMPASS;
		} else {
			debug::show_message("Warning: unknown gps search type given.");
		}
	}
	index++;

    if (args.size() > index) std::istringstream(args[index]) >> options.dimensions;
    index++;
    if (args.size() > index) std::istringstream(args[index]) >> options.explorationCoefficient;
    index++;
    if (args.size() > index) std::istringstream(args[index]) >> options.newSearchPointCoefficient;
    index++;
    if (args.size() > index) std::istringstream(args[index]) >> options.minimumVisitsBeforeChildCreation;
    index++;
    if (args.size() > index) std::istringstream(args[index]) >> options.minimumChildCreationDistance;
    index++;

    if (args.size() < index) {
    	debug::show_message("Warning: not enough arguments given for gps search. Some default values are used.");
    } else if (args.size() > index) {
    	debug::show_message("Warning: too many arguments given for gps search. Extra arguments are ignored.");
    }

    if (options.newSearchPointCoefficient <= 0) {
    	options.disableGpsSearch = true;
    } else {
    	options.disableGpsSearch = false;
    }

    return std::make_unique<solver::GpsStepGeneratorFactory>(solver, options);
}


std::unique_ptr<solver::StepGeneratorFactory> NnRolloutParser::parse(solver::Solver *solver,
        std::vector<std::string> args) {
    long maxNnComparisons;
    double maxNnDistance;
    std::istringstream(args[1]) >> maxNnComparisons;
    std::istringstream(args[2]) >> maxNnDistance;
    return std::make_unique<solver::NnRolloutFactory>(solver, maxNnComparisons, maxNnDistance);
}
std::unique_ptr<solver::StepGeneratorFactory> DefaultRolloutParser::parse(solver::Solver *solver,
        std::vector<std::string> args) {
    long maxNSteps;
    std::istringstream(args[1]) >> maxNSteps;
    return std::make_unique<solver::DefaultRolloutFactory>(solver, maxNSteps);
}

StagedParser::StagedParser(ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *allParsers) :
            allParsers_(allParsers) {
}
std::unique_ptr<solver::StepGeneratorFactory> StagedParser::parse(solver::Solver *solver,
        std::vector<std::string> args) {
    std::vector<std::unique_ptr<solver::StepGeneratorFactory>> factories;
    for (auto it = args.begin() + 1; it != args.end(); it++) {
        factories.push_back(allParsers_->parse(solver, *it));
    }
    return std::make_unique<solver::StagedStepGeneratorFactory>(std::move(factories));
}

DefaultHeuristicParser::DefaultHeuristicParser(ModelWithProgramOptions *model) :
        heuristic_() {
    heuristic_ = [model] (solver::HistoryEntry const *entry,
            solver::State const *state, solver::HistoricalData const *data) {
        return model->getDefaultHeuristicValue(entry, state, data);
    };
}
solver::HeuristicFunction DefaultHeuristicParser::parse(
        solver::Solver */*solver*/, std::vector<std::string> /*args*/) {
    return heuristic_;
}
solver::HeuristicFunction ZeroHeuristicParser::parse(
        solver::Solver */*solver*/, std::vector<std::string> /*args*/) {
    return [] (solver::HistoryEntry const *, solver::State const *,
            solver::HistoricalData const *) {
        return 0;
    };
}

BasicSearchParser::BasicSearchParser(
        ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *generatorParsers,
        ParserSet<solver::HeuristicFunction> *heuristicParsers, std::string heuristicString) :
            generatorParsers_(generatorParsers),
            heuristicParsers_(heuristicParsers),
            heuristicString_(heuristicString) {
}

std::unique_ptr<solver::SearchStrategy> BasicSearchParser::parse(solver::Solver *solver,
        std::vector<std::string> args) {
    std::unique_ptr<solver::StepGeneratorFactory> factory = nullptr;
    std::string heuristicString = heuristicString_;
    if (args[0] != "basic") {
        factory = generatorParsers_->parse(solver, args);
    } else {
        factory = generatorParsers_->parse(solver, args[1]);
        if (args.size() > 2) {
            heuristicString = args[2];
        }
    }

    return std::make_unique<solver::BasicSearchStrategy>(solver, std::move(factory),
            heuristicParsers_->parse(solver, heuristicString));
}

Exp3Parser::Exp3Parser(ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParsers) :
        allParsers_(allParsers) {
}
std::unique_ptr<solver::SearchStrategy> Exp3Parser::parse(solver::Solver *solver,
        std::vector<std::string> args) {
    std::vector<std::unique_ptr<solver::SearchStrategy>> strategies;
    std::vector<std::string>::iterator it = args.begin() + 1;
    double strategyExplorationCoefficient;
    std::istringstream(*it) >> strategyExplorationCoefficient;
    it++;
    for (; it != args.end(); it++) {
        strategies.push_back(allParsers_->parse(solver, *it));
    }
    return std::make_unique<solver::MultipleStrategiesExp3>(solver, strategyExplorationCoefficient,
            std::move(strategies));
}

std::unique_ptr<solver::EstimationStrategy> AverageEstimateParser::parse(solver::Solver */*solver*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::EstimationFunction>(
            std::function<double(solver::BeliefNode const *)>(solver::estimators::average));
}
std::unique_ptr<solver::EstimationStrategy> MaxEstimateParser::parse(solver::Solver */*solver*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::EstimationFunction>(solver::estimators::max);
}
std::unique_ptr<solver::EstimationStrategy> RobustEstimateParser::parse(solver::Solver */*solver*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::EstimationFunction>(solver::estimators::robust);
}


std::unique_ptr<solver::SelectRecommendedActionStrategy> MaxRecommendedActionStrategyParser::parse(solver::Solver * /*solver*/,
        std::vector<std::string> /*args*/) {
    return std::make_unique<solver::MaxRecommendedActionStrategy>();
}

std::unique_ptr<solver::SelectRecommendedActionStrategy> GpsMaxRecommendedActionStrategyParser::parse(solver::Solver * /*solver*/,
        std::vector<std::string> args) {

	using solver::choosers::GpsMaxRecommendationOptions;
	GpsMaxRecommendationOptions options;

	// We only need to parse the options that are actually relevant for the max chooser.

	size_t index = 1;

	if (args.size() > index) {
		if (args[index] == "golden") {
			options.searchType = GpsMaxRecommendationOptions::GOLDEN;
		} else if (args[index] == "compass") {
			options.searchType = GpsMaxRecommendationOptions::COMPASS;
		} else {
			debug::show_message("Warning: unknown gps search type given.");
		}
	}
	index++;

    if (args.size() > index) std::istringstream(args[index]) >> options.dimensions;
    index++;

    bool enableGpsSearch = true;
    if (args.size() > index) std::istringstream(args[index]) >> enableGpsSearch;
    options.disableGpsSearch = !enableGpsSearch;
    index++;

    if (args.size() < index) {
    	debug::show_message("Warning: not enough arguments given for gps max recommendation. Some default values are used.");
    } else if (args.size() > index) {
    	debug::show_message("Warning: too many arguments given for gps max recommendation. Extra arguments are ignored.");
    }

    return std::make_unique<solver::GpsMaxRecommendedActionStrategy>(options);
}



} /* namespace shared */
