#include "global.hpp"

namespace abt {
// Used for printing values in GDB
std::stringstream *get_sstr() {
    static std::stringstream sstr;
    return &sstr;
}

void reset_sstr() {
    get_sstr()->str("");
    get_sstr()->clear();
}
} /* namespace global */
