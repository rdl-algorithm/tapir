#ifndef OPTION_PARSER_HPP_
#define OPTION_PARSER_HPP_

#include "global.hpp"

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <tclap/CmdLine.h>
#include <inih/ini.hpp>

#include "global.hpp"

namespace options {
struct BaseOptions {
};

class BaseArg {
public:
    BaseArg() = default;
    virtual ~BaseArg() = default;
    _NO_COPY_OR_MOVE(BaseArg);

    virtual TCLAP::Arg &getArg() = 0;
};


class BaseOption {
public:
    BaseOption(std::string section, std::string name) :
            section_(section),
            name_(name),
            aliases_() {
    }
    virtual ~BaseOption() = default;

    std::string getSection() {
        return section_;
    }

    std::string getName() {
        return name_;
    }

    void addAlias(std::unique_ptr<BaseArg> alias) {
        aliases_.push_back(std::move(alias));
    }

    void printAliases(std::ostream &os) {
        if (!aliases_.empty()) {
            os << "Aliases for " << getSection() << "." << getName() << ":" << std::endl;
            for (auto const &alias : aliases_) {
                std::string shortOpt = alias->getArg().getFlag();
                if (!shortOpt.empty()) {
                    os << "-" << shortOpt << std::endl;
                }
                std::string longOpt = alias->getArg().getName();
                if (!longOpt.empty()) {
                    os << "--" << longOpt << std::endl;
                }
            }
        }
    }

    virtual void parse(std::string s) = 0;
    virtual void setDefault() = 0;
    virtual bool isSet() = 0;
    virtual bool isDefaulted() = 0;
private:
    std::string section_;
    std::string name_;
    std::vector<std::unique_ptr<BaseArg>> aliases_;
};

class OptionParser {
public:
    OptionParser(std::string const &message);
    ~OptionParser() = default;
    _NO_COPY_OR_MOVE(OptionParser);

    template<typename ValueType, typename OptionsType>
    void addOption(std::string section, std::string name, ValueType OptionsType::*optionPtr);

    template<typename ValueType, typename OptionsType>
    void addOptionWithDefault(std::string section, std::string name,
            ValueType OptionsType::*optionPtr, ValueType defaultValue);

    template<typename ArgType, typename ValueType, typename OptionsType, typename ... Args>
    void addCmdAlias(std::string section, std::string name, ValueType OptionsType::*optionPtr,
            Args&& ... args);

    template<typename ValueType, typename OptionsType> void addValueArg(std::string section,
            std::string name, ValueType OptionsType::*optionPtr, std::string shortOpt,
            std::string longOpt, std::string description, std::string typeDescription);

    template<typename OptionsType> void addSwitchArg(std::string section, std::string name,
            bool OptionsType::*optionPtr, std::string shortOpt, std::string longOpt,
            std::string description, bool switchValue);

    BaseOptions *getOptions();
    void setOptions(BaseOptions *options);

    void initialize();
    void parseCmdLine(int argc, char const *argv[]);
    void parseCfgFile(std::string path);
    void finalize();
private:
    static int iniHandler(void *user, char const *section, char const *name, char const *value);

    BaseOptions *options_;
    TCLAP::CmdLine cmdLine_;
    std::unordered_map<std::string, std::unordered_map<std::string, std::unique_ptr<BaseOption>>>optionsMap_;
};

// Generic parser.
template<typename ValueType> struct Parser {
    static ValueType parse(std::string s) {
        ValueType value;
        std::istringstream(s) >> value;
        return value;
    }
};

// Specialised parser for std::string
template<> struct Parser<std::string> {
    static std::string parse(std::string s) {
        return s;
    }
};

// Specialised parser for bool
template<> struct Parser<bool> {
    static bool parse(std::string s) {
        bool value;
        std::istringstream(s) >> std::boolalpha >> value;
        return value;
    }
};

template<typename ValueType, typename OptionsType>
class Option: public BaseOption {
public:
    _NO_COPY_OR_MOVE(Option);
    Option(std::string section, std::string name, OptionParser *parser,
            ValueType OptionsType::*optionPtr, std::unique_ptr<ValueType> defaultValue = nullptr) :
            BaseOption(section, name),
            parser_(parser),
            optionPtr_(optionPtr),
            isSet_(false),
            isDefaulted_(false),
            defaultValue_(std::move(defaultValue)) {
    }
    ~Option() = default;

    virtual bool isSet() override {
        return isSet_;
    }

    virtual bool isDefaulted() override {
        return isDefaulted_;
    }

    virtual void parse(std::string s) override {
        setValue(Parser<ValueType>::parse(s));
    }

    virtual void setDefault() override {
        if (defaultValue_ != nullptr) {
            setValue(*defaultValue_);
            isDefaulted_ = true;
        }
    }

    void setValue(ValueType value) {
        static_cast<OptionsType &>(*parser_->getOptions()).*optionPtr_ = value;
        isSet_ = true;
        isDefaulted_ = false;
    }

private:
    OptionParser *parser_;
    ValueType OptionsType::*optionPtr_;
    bool isSet_;
    bool isDefaulted_;
    std::unique_ptr<ValueType> defaultValue_;
};

template<typename ArgType, typename ValueType, typename OptionsType>
class VisitingArg: public BaseArg, public TCLAP::Visitor {
public:
    _NO_COPY_OR_MOVE(VisitingArg);
    template<typename ... Args>
    VisitingArg(OptionParser *parser, Option<ValueType, OptionsType> *option, Args&&... args) :
            parser_(parser), option_(option), arg_(std::forward<Args>(args)..., this) {
    }
    ~VisitingArg() = default;

    virtual TCLAP::Arg &getArg() override {
        return arg_;
    }

    virtual void visit() override {
        option_->setValue(arg_.getValue());
    }

private:
    OptionParser *parser_;
    Option<ValueType, OptionsType> *option_;
    ArgType arg_;
};

template<typename ValueType, typename OptionsType>
void OptionParser::addOption(std::string section, std::string name,
        ValueType OptionsType::*optionPtr) {
    optionsMap_[section][name] = std::make_unique<Option<ValueType, OptionsType>>(section, name,
            this, optionPtr);
}

template<typename ValueType, typename OptionsType>
void OptionParser::addOptionWithDefault(std::string section, std::string name,
        ValueType OptionsType::* optionPtr, ValueType defaultValue) {
    optionsMap_[section][name] = std::make_unique<Option<ValueType, OptionsType>>(section, name,
            this, optionPtr, std::make_unique<ValueType>(defaultValue));
}

template<typename ArgType, typename ValueType, typename OptionsType, typename ... Args>
void OptionParser::addCmdAlias(std::string section, std::string name,
        ValueType OptionsType::*/*optionPtr*/, Args&& ... args) {
    Option<ValueType, OptionsType> *option =
            static_cast<Option<ValueType, OptionsType> *>(optionsMap_[section][name].get());
    std::unique_ptr<BaseArg> visitor = (
            std::make_unique<VisitingArg<ArgType, ValueType, OptionsType>>(
                    this, option, std::forward<Args>(args)...));
    cmdLine_.add(visitor->getArg());
    option->addAlias(std::move(visitor));
}

template<typename ValueType, typename OptionsType> void OptionParser::addValueArg(
        std::string section, std::string name, ValueType OptionsType::*optionPtr,
        std::string shortOpt, std::string longOpt,
        std::string description, std::string typeDescription) {
    addCmdAlias<TCLAP::ValueArg<ValueType>, ValueType, OptionsType>(section, name, optionPtr,
            shortOpt, longOpt, description, false, ValueType(), typeDescription);
}

template<typename OptionsType> void OptionParser::addSwitchArg(std::string section, std::string name,
        bool OptionsType::*optionPtr, std::string shortOpt, std::string longOpt,
        std::string description,  bool switchValue) {
    addCmdAlias<TCLAP::SwitchArg, bool, OptionsType>(section, name, optionPtr,
            shortOpt, longOpt, description, !switchValue);
}
} /* namespace options */

#endif /* OPTION_PARSER_HPP_ */
