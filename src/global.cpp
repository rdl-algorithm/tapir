#include "global.hpp"

#include <iostream>
#include <unistd.h>

#include "solver/abstract-problem/Point.hpp"

namespace abt {
void printDouble(double value, std::ostream &os, bool showpos, int precision,
        int numDigitsBeforePoint) {
    std::streamsize oldPrecision = os.precision(precision);
    std::streamsize width = os.width(precision + numDigitsBeforePoint + 2);
    std::ios_base::fmtflags flags = std::ios_base::fixed;
    if (showpos) {
        flags |= std::ios_base::showpos;
    }
    flags = os.flags(flags);
    os << value;
    os.flags(flags);
    os.width(width);
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
