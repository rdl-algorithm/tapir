#ifndef GEOMETRY_UTILITIES_HPP_
#define GEOMETRY_UTILITIES_HPP_

#include <cmath>

namespace geometry {
double normalizeAngle(double angle) {
    double numRotations;
    angle = std::modf(angle, &numRotations);
    if (angle < -0.5) {
        angle += 1;
    } else if (angle > 0.5) {
        angle -= 1;
    }
    return angle;
}
} /* namespace geometry */

#endif /* GEOMETRY_UTILITIES_HPP_ */
