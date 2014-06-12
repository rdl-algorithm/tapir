#ifndef STRATEGY_PARSERS_HPP_
#define STRATEGY_PARSERS_HPP_

#include <unordered_map>

#include "solver/cached_values.hpp"
#include "solver/belief-estimators/estimators.hpp"
#include "solver/search/search_interface.hpp"

template<typename TargetType> class Parser;
class ModelWithProgramOptions;

std::vector<std::string> split_function(std::string text);

template<typename TargetType>
class ParserSet {
public:
    ParserSet() :
                parsers_(),
                defaultParser_(nullptr) {
    }
    virtual ~ParserSet() = default;

    /**  Adds the given parser. */
    void addParser(std::string name, std::unique_ptr<Parser<TargetType>> parser) {
        parsers_.emplace(name, std::move(parser));
    }

    void setDefaultParser(std::unique_ptr<Parser<TargetType>> parser) {
        defaultParser_ = std::move(parser);
    }

    /** Returns the parser associated with the strategy of the given name. */
    Parser<TargetType> *getParser(std::string name) {
        return parsers_.at(name).get();
    }

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
    std::unordered_map<std::string, std::unique_ptr<Parser<TargetType>>> parsers_;
    std::unique_ptr<Parser<TargetType>> defaultParser_;
};

template<typename TargetType>
class Parser {
public:
    Parser() = default;
    virtual ~Parser() = default;

    /** Creates a new strategy from a vector of strings representing its
     * arguments.
     */
    virtual TargetType parse(solver::Solver *solver, std::vector<std::string> args) = 0;
};

class UcbParser: public Parser<std::unique_ptr<solver::StepGeneratorFactory>> {
public:
    UcbParser() = default;
    virtual ~UcbParser() = default;
    virtual std::unique_ptr<solver::StepGeneratorFactory> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};
class NnRolloutParser: public Parser<std::unique_ptr<solver::StepGeneratorFactory>> {
public:
    NnRolloutParser() = default;
    virtual ~NnRolloutParser() = default;
    virtual std::unique_ptr<solver::StepGeneratorFactory> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};
class DefaultRolloutParser: public Parser<std::unique_ptr<solver::StepGeneratorFactory>> {
public:
    DefaultRolloutParser() = default;
    virtual ~DefaultRolloutParser() = default;
    virtual std::unique_ptr<solver::StepGeneratorFactory> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};
class StagedParser: public Parser<std::unique_ptr<solver::StepGeneratorFactory>> {
public:
    StagedParser(ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *allParsers);
    virtual ~StagedParser() = default;
    _NO_COPY_OR_MOVE(StagedParser);

    virtual std::unique_ptr<solver::StepGeneratorFactory> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
private:
    ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *allParsers_;
};

class DefaultHeuristicParser: public Parser<solver::Heuristic> {
public:
    DefaultHeuristicParser(ModelWithProgramOptions *model);
    virtual ~DefaultHeuristicParser() = default;
    virtual solver::Heuristic parse(solver::Solver */*solver*/,
            std::vector<std::string> args) override;

private:
    solver::Heuristic heuristic_;
};

class ZeroHeuristicParser: public Parser<solver::Heuristic> {
public:
    ZeroHeuristicParser() = default;
    virtual ~ZeroHeuristicParser() = default;
    virtual solver::Heuristic parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};

class BasicSearchParser: public Parser<std::unique_ptr<solver::SearchStrategy>> {
public:
    BasicSearchParser(ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *generatorParsers,
            ParserSet<solver::Heuristic> *heuristicParsers, std::string heuristicString);
    virtual ~BasicSearchParser() = default;
    _NO_COPY_OR_MOVE(BasicSearchParser);

    virtual std::unique_ptr<solver::SearchStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
private:
    ParserSet<std::unique_ptr<solver::StepGeneratorFactory>> *generatorParsers_;
    ParserSet<solver::Heuristic> *heuristicParsers_;
    std::string heuristicString_;
};

class Exp3Parser: public Parser<std::unique_ptr<solver::SearchStrategy>> {
public:
    Exp3Parser(ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParsers);
    virtual ~Exp3Parser() = default;
    _NO_COPY_OR_MOVE(Exp3Parser);

    virtual std::unique_ptr<solver::SearchStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
private:
    ParserSet<std::unique_ptr<solver::SearchStrategy>> *allParsers_;
};

class AverageEstimateParser: public Parser<std::unique_ptr<solver::EstimationStrategy>> {
public:
    AverageEstimateParser() = default;
    virtual ~AverageEstimateParser() = default;
    virtual std::unique_ptr<solver::EstimationStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};

class MaxEstimateParser: public Parser<std::unique_ptr<solver::EstimationStrategy>> {
public:
    MaxEstimateParser() = default;
    virtual ~MaxEstimateParser() = default;
    virtual std::unique_ptr<solver::EstimationStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};

class RobustEstimateParser: public Parser<std::unique_ptr<solver::EstimationStrategy>> {
public:
    RobustEstimateParser() = default;
    virtual ~RobustEstimateParser() = default;
    virtual std::unique_ptr<solver::EstimationStrategy> parse(solver::Solver *solver,
            std::vector<std::string> args) override;
};
#endif /* STRATEGY_PARSERS_HPP_ */
