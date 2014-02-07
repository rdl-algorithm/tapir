#include "SimpleVectorL1.hpp"

#include <memory>
#include <vector>

#include "defs.hpp"

namespace solver {
SimpleVectorL1::SimpleVectorL1(std::vector<double> values) :
    values_(values) {
}

std::unique_ptr<Point> SimpleVectorL1::copy() const {
    return std::make_unique<SimpleVectorL1>(values_);
}

double SimpleVectorL1::distanceTo(Point const &otherPoint) const {
    SimpleVectorL1 const &other = static_cast<SimpleVectorL1 const &>(otherPoint);
    std::vector<double>::const_iterator i1 = values_.begin();
    std::vector<double>::const_iterator i2 = other.values_.begin();
    double distance = 0;
    for (; i1 < values_.end(); i1++, i2++) {
        distance += std::abs(*i1 - *i2);
    }
    return distance;
}

std::vector<double> SimpleVectorL1::asVector() const {
    return values_;
}

} /* namespace solver */
