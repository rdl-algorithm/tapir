#include "options/option_parser.hpp"

#include "global.hpp"

namespace options {
OptionParser::OptionParser(std::string const &message) :
        options_(nullptr), cmdLine_(message), optionsMap_() {
}

BaseOptions *OptionParser::getOptions() {
    return options_;
}

void OptionParser::setOptions(BaseOptions *options) {
    options_ = options;
}

void OptionParser::initialize() {
    for (auto &entry : optionsMap_) {
        for (auto &entry2 : entry.second) {
            entry2.second->setDefault();
        }
    }
}

void OptionParser::parseCmdLine(int argc, char const *argv[]) {
    cmdLine_.parse(argc, argv);
}

void OptionParser::parseCfgFile(std::string path) {
    ini_parse(path.c_str(), OptionParser::iniHandler, this);
}

int OptionParser::iniHandler(void *user, char const *section, char const *name, char const *value) {
    OptionParser *parser = static_cast<OptionParser *>(user);
    try {
        BaseOption *option = parser->optionsMap_.at(section).at(name).get();
        if (option->isDefaulted() || !option->isSet()) {
            option->parse(value);
        }
    } catch (std::out_of_range const &oor) {
        std::ostringstream message;
        message << "ERROR: Invalid option in config file: " << section << "." << name << std::endl;
        debug::show_message(message.str());
        std::exit(2);
    }
    return 1;
}

void OptionParser::finalize() {
    for (auto &entry : optionsMap_) {
        for (auto &entry2 : entry.second) {
            if (!entry2.second->isSet()) {
                std::ostringstream message;
                message << "ERROR: Missing mandatory option ";
                message << entry.first << "." << entry2.first << std::endl;
                entry2.second->printAliases(message);
                debug::show_message(message.str());
                std::exit(1);
            }
        }
    }
}
} /* namespace options */
