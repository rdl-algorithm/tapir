#include "utilities.hpp"

#include <cmath>

namespace geometry {
double normalizeTurn(double turn) {
    double numRotations;
    turn = std::modf(turn, &numRotations);
    if (turn <= -0.5) {
        turn += 1;
    } else if (turn > 0.5) {
        turn -= 1;
    }
    return turn;
}
} /* namespace geometry */
