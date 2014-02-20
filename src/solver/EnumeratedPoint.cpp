#include "EnumeratedPoint.hpp"

#include "Point.hpp"

namespace solver {
bool EnumeratedPoint::equals(Point const &otherPoint) const {
    EnumeratedPoint const &other =
            static_cast<EnumeratedPoint const &>(otherPoint);
    return getCode() == other.getCode();
}

std::size_t EnumeratedPoint::hash() const {
    return getCode();
}

void EnumeratedPoint::print(std::ostream &os) const {
    os << getCode();
}
} /* namespace solver */
