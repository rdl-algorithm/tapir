/** file: global.cpp
 *
 * Implementation for show_message() - a great place for breakpoints!
 */
#include "global.hpp"

#include <iostream>
#include <unistd.h>

#include "solver/abstract-problem/Point.hpp"

namespace debug {
void show_message(std::string message, bool print, bool bp_branch) {
    if (print) {
        std::cerr << message << std::endl;
    }
    if (bp_branch) {
        std::cerr << "";
    }
}

template std::string to_string<solver::Point &>(solver::Point &);
} /* namespace debug */
