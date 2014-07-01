//    Copyright (c) 2014 Dimitri Klimenko
//
//    Permission is hereby granted, free of charge, to any person obtaining a copy
//    of this software and associated documentation files (the "Software"), to deal
//    in the Software without restriction, including without limitation the rights
//    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//    copies of the Software, and to permit persons to whom the Software is
//    furnished to do so, subject to the following conditions:
//
//    The above copyright notice and this permission notice shall be included in
//    all copies or substantial portions of the Software.
//
//    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//    THE SOFTWARE.


/** file: option_parser.cpp
 *
 * Contains the implementations for the non-template methods used in parsing options.
 */
#include "options/option_parser.hpp"

namespace options {
OptionParsingException::OptionParsingException(std::string const &message) :
        message_(message) {
}

OptionParser::OptionParser(std::string const &message) :
        options_(nullptr),
        cmdLine_(message),
        optionsMap_() {
}

BaseOptions *OptionParser::getOptions() {
    return options_;
}

void OptionParser::setOptions(BaseOptions *options) {
    options_ = options;
    for (auto &entry : optionsMap_) {
        for (auto &entry2 : entry.second) {
            entry2.second->reset();
            entry2.second->setDefault(options_);
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
            option->parse(parser->options_, value);
        }
    } catch (std::out_of_range const &oor) {
        std::ostringstream message;
        message << "ERROR: Invalid option in config file: " << section << "." << name << std::endl;
        throw OptionParsingException(message.str());
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
                throw OptionParsingException(message.str());
            }
        }
    }
}
} /* namespace options */
