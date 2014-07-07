/** @file parsers.hpp
 *
 * Provides some convenient functionality for parsing heuristics, search strategies and estimators
 * for belief values.
 */
#ifndef SHARED_PARSERS_HPP_
#define SHARED_PARSERS_HPP_

#include <unordered_map>

#include "solver/cached_values.hpp"
#include "solver/abstract-problem/heuristics/HeuristicFunction.hpp"
#include "solver/belief-estimators/estimators.hpp"
#include "solver/search/search_interface.hpp"

namespace shared {
class ModelWithProgramOptions;

/** Splits the given string as a function, i.e. having the form functionName(arg0, arg1, ...).
 *
 * This function does basic text parsing; nested brackets are assumed to be part of the arguments
 * if they occur, and these can later be parsed in turn if the parser expects to see nested
 * functions.
 */
std::vector<std::string> split_function(std::string text);

/** A class to parse strings into arbitrary constructs for the Solver to use. */
template<typename TargetType>
class Parser {
public:
    Parser() = default;
    virtual ~Parser() = default;

    /** Creates a new strategy from a vector of arguments (as strings). */
    virtual TargetType parse(solver::Solver *solver, std::vector<std::string> args) = 0;
};

/** A class to hold a set of parsers of the same type, each of which will have an associated
 * string identifier.
 */
template<typename TargetType>
class ParserSet {
public:
    /** Creates a new, empty set of parsers. */
    ParserSet() :
            parsers_(),
            defaultParser_(nullptr) {
    }
    virtual ~ParserSet() = default;

    /**  Adds the given parser to this set. */
    void addParser(std::string name, std::unique_ptr<Parser<TargetType>> parser) {
        parsers_.emplace(name, std::move(parser));
    }

    /** Sets the default parser for this set - this one will be used if the desired parser
     * cannot be found.
     */
    void setDefaultParser(std::unique_ptr<Parser<TargetType>> parser) {
        defaultParser_ = std::move(parser);
    }

    /** Returns the parser associated with the strategy of the given name. */
    Parser<TargetType> *getParser(std::string name) {
        return parsers_.at(name).get();
    }

    /** Returns a target object parsed from the given vector of strings.
     *
     * The first element of the vector will be taken as the parser type, or, if the type is not
     * found and a default parser has been set, the default parser will be used.
     */
    TargetType parse(solver::Solver *solver, std::vector<std::string> argsVector) {
        std::string targetType = argsVector[0];
        try {
            return getParser(targetType)->parse(solver, argsVector);
        } catch (std::out_of_range const &error) {
            if (defaultParser_ == nullptr) {
                std::ostringstream message;
                message << "ERROR: Invalid target type \"" << targetType;
                message << "\"" << std::endl;
                debug::show_message(message.str());
            } else {
                return defaultParser_->parse(solver, argsVector);
            }
        }
        return nullptr;
    }

    /** Returns a new target object parsed from the given string. */
    TargetType parse(solver::Solver *solver, std::string targetString) {
        return parse(solver, split_function(targetString));
    }

private:
    /** The set of parsers, to be looked up by their string IDs. */
    std::unordered_map<std::string, std::unique_ptr<Parser<TargetType>>> parsers_;

    /** The default parser, if applicable. */
    std::unique_ptr<Parser<TargetType>> defaultParser_;
};

/** A parser for UcbStepGeneratorFactory instances. */
class UcbParser: public Parser<std::unique_ptr<solver::StepGeneratorFactory>> {
public:
    UcbParser() = default;
    virtual ~UcbParser() = default;
    virtual std::unique_ptr<solver::StepGeneratorFactory> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};

/** A parser for NnRolloutFactory instances. */
class NnRolloutParser: public Parser<std::unique_ptr<solver::StepGeneratorFactory>> {
public:
    NnRolloutParser() = default;
    virtual ~NnRolloutParser() = default;
    virtual std::unique_ptr<solver::StepGeneratorFactory> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};

/** A parser for DefaultRolloutFactory instances. */
class DefaultRolloutParser: public Parser<std::unique_ptr<solver::StepGeneratorFactory>> {
public:
    DefaultRolloutParser() = default;
    virtual ~DefaultRolloutParser() = default;
    virtual std::unique_ptr<solver::StepGeneratorFactory> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};

/** A parser for StagedStepGeneratorFactory instances - this can be used to put several different
 * ways of generating steps in sequence, e.g. UCB followed by a rollout phase.
 */
class StagedParser: public Parser<std::unique_ptr<solver::StepGeneratorFactory>> {
public:
    /** Creates a new StagedParser that will use the given set of parsers for StepGeneratorFactory
     * instances in order to parse the invididual stages.
     */
    StagedParser(ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *allParsers);
    virtual ~StagedParser() = default;
    _NO_COPY_OR_MOVE(StagedParser);

    virtual std::unique_ptr<solver::StepGeneratorFactory> parse(solver::Solver *solver,
            std::vector<std::string> args) override;

private:
    /** The set of parsers to use for parsing the individual instances that make up the staged
     * strategy.
     */
    ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *allParsers_;
};

/** A parser for the default heuristic, which uses the virtual method
 * ModelWithProgramOptions::getDefaultHeuristicValue.
 */
class DefaultHeuristicParser: public Parser<solver::HeuristicFunction> {
public:
    /** Creates a new DefaultHeuristicParser associated with the given model. */
    DefaultHeuristicParser(ModelWithProgramOptions *model);
    virtual ~DefaultHeuristicParser() = default;
    virtual solver::HeuristicFunction parse(solver::Solver */*solver*/,
            std::vector<std::string> args) override;

private:
    /** The heuristic to be used - a functional wrapper for
     * ModelWithProgramOptions::getDefaultHeuristicValue for the given model instance.
     */
    solver::HeuristicFunction heuristic_;
};

/** A parser that returns a trivial heuristic function, which simply returns zero. */
class ZeroHeuristicParser: public Parser<solver::HeuristicFunction> {
public:
    ZeroHeuristicParser() = default;
    virtual ~ZeroHeuristicParser() = default;
    virtual solver::HeuristicFunction parse(solver::Solver *solver, std::vector<std::string> args)
            override;
};

/** The default parser for search strategies.
 *
 * The strategy can be expressed as "stepper", in which case the standard heuristic function will
 * be used, or it can also be expressed as "basic(stepper, heuristic)", which allows for a
 * choice between different heuristics as compared to other strategies.
 */
class BasicSearchParser: public Parser<std::unique_ptr<solver::SearchStrategy>> {
public:
    /** Creates a new parser for basic search strategies, which will use the given set of
     * parsers for StepGeneratorFactory instances and the given set of heuristic parsers, using
     * heuristicString as the default heuristic.
     */
    BasicSearchParser(ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *generatorParsers,
            ParserSet<solver::HeuristicFunction> *heuristicParsers, std::string heuristicString);
    virtual ~BasicSearchParser() = default;
    _NO_COPY_OR_MOVE(BasicSearchParser);
    virtual std::unique_ptr<solver::SearchStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;

private:
    /** The set of parsers for parsing StepGeneratorFactory instances. */
    ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *generatorParsers_;
    /** The set of parsers for parsing HeuristicFunctions. */
    ParserSet<solver::HeuristicFunction> *heuristicParsers_;

    /** The default heuristic, as a string. */
    std::string heuristicString_;
};

/** A parser for Exp3 meta-strategies. */
class Exp3Parser: public Parser<std::unique_ptr<solver::SearchStrategy>> {
public:
    /** Creates a parser for Exp3 strategies using the given set of parsers to parse the
     * individual strategies in the Exp3 meta-strategy.
     */
    Exp3Parser(ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParsers);
    virtual ~Exp3Parser() = default;
    _NO_COPY_OR_MOVE(Exp3Parser);
    virtual std::unique_ptr<solver::SearchStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;

private:
    /** The set of parsers for parsing individual strategies. */
    ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParsers_;
};

/** A parser for the default estimation method, using the average. */
class AverageEstimateParser: public Parser<std::unique_ptr<solver::EstimationStrategy>> {
public:
    AverageEstimateParser() = default;
    virtual ~AverageEstimateParser() = default;
    virtual std::unique_ptr<solver::EstimationStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};

/** A parser for the standard Bellman estimation method, where the value of a belief node is
 * estimated based on the highest Q(b, a) value.
 */
class MaxEstimateParser: public Parser<std::unique_ptr<solver::EstimationStrategy>> {
public:
    MaxEstimateParser() = default;
    virtual ~MaxEstimateParser() = default;
    virtual std::unique_ptr<solver::EstimationStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};

/** A parser that estimates the value of a belief node based on the action with the highest
 * visit count from that node.
 */
class RobustEstimateParser: public Parser<std::unique_ptr<solver::EstimationStrategy>> {
public:
    RobustEstimateParser() = default;
    virtual ~RobustEstimateParser() = default;
    virtual std::unique_ptr<solver::EstimationStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};
} /* namespace shared */

#endif /* SHARED_PARSERS_HPP_ */
