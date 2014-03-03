#include "global.hpp"

#include <iostream>

#include "solver/geometry/Point.hpp"

namespace debug {
//void show_message(char const *message, bool print) {
//    if (print) {
//        std::cerr << message << std::endl;
//    }
//}
void show_message(std::string message, bool print) {
    if (print) {
        std::cerr << message << std::endl;
    }
}

template std::string to_string<solver::Point &>(solver::Point &);
} /* namespace debug */
