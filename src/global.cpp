/** @file global.cpp
 *
 * Implementation for show_message() - a great place for breakpoints!
 */
#include "global.hpp"

#include <iostream>
#include <unistd.h>

#include "solver/abstract-problem/Point.hpp"

namespace abt {
std::string get_current_directory() {
    char *buffer = getcwd(nullptr, 0);
    if (buffer == nullptr) {
        debug::show_message("ERROR: Failed to get current path.");
        std::exit(4);
    }
    std::string dir(buffer);
    free(buffer);
    return dir;
}

void change_directory(std::string &dir) {
    if (chdir(dir.c_str())) {
        std::ostringstream oss;
        oss << "ERROR: Failed to change path to " << dir;
        debug::show_message(oss.str());
        std::exit(3);
    }
}
} /* namespace abt */


namespace debug {
void show_message(std::string message, bool print, bool bp_branch) {
    if (print) {
        std::cerr << message << std::endl;
    }
    if (bp_branch) {
        std::cerr << "";
    }
}

/** Instantiation of the to_string template for easier printing of points in GDB. */
template std::string to_string<solver::Point &>(solver::Point &);
} /* namespace debug */
