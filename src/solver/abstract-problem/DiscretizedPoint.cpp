#include "DiscretizedPoint.hpp"

#include "Point.hpp"

namespace solver {
bool DiscretizedPoint::equals(Point const &otherPoint) const {
    DiscretizedPoint const &other =
            static_cast<DiscretizedPoint const &>(otherPoint);
    return getBinNumber() == other.getBinNumber();
}

std::size_t DiscretizedPoint::hash() const {
    return getBinNumber();
}

void DiscretizedPoint::print(std::ostream &os) const {
    os << getBinNumber();
}
} /* namespace solver */
