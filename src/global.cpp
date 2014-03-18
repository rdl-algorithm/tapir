#include "global.hpp"

#include <iostream>
#include <unistd.h>

#include "solver/abstract-problem/Point.hpp"

namespace abt {
void print_double(double value, std::ostream &os, int width,
        int precision,
        std::ios_base::fmtflags flags,
        std::ostream::char_type fillCh) {
    std::streamsize oldPrecision = os.precision(precision);
    std::streamsize oldWidth = os.width(width);
    std::ostream::char_type oldFillCh = os.fill(fillCh);
    std::ios_base::fmtflags oldflags = os.flags(flags);
    os << value;
    os.flags(oldflags);
    os.fill(oldFillCh);
    os.width(oldWidth);
    os.precision(oldPrecision);
}
}

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
