#include "global.hpp"

#include <iostream>

#include "solver/geometry/Point.hpp"

namespace debug {
void show_message(std::string message, bool print, bool bp_branch) {
    if (print) {
        std::cerr << message << std::endl;
    }
    if (bp_branch) {
        // Do nothing! This is a nice place for a breakpoint!
    }
}

template std::string to_string<solver::Point &>(solver::Point &);
} /* namespace debug */
