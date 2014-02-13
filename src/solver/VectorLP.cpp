#include "VectorLP.hpp"

#include <cmath>

#include <initializer_list>
#include <memory>
#include <vector>

#include "defs.hpp"
#include "Vector.hpp"

namespace solver {
VectorLP::VectorLP(std::initializer_list<double> values, double p) :
        p_(p), values_(values) {
}

VectorLP::VectorLP(std::vector<double> values, double p) :
        p_(p), values_(values) {
}

bool VectorLP::equals(Point const &otherPoint) const {
    VectorLP const &other = static_cast<VectorLP const &>(otherPoint);
    std::vector<double>::const_iterator i1 = values_.begin();
    std::vector<double>::const_iterator i2 = other.values_.begin();
    for (; i1 < values_.end(); i1++, i2++) {
        if (*i1 != *i2) {
            return false;
        }
    }
    return true;
}

std::size_t VectorLP::hash() const {
    std::size_t hashValue = 0;
    for (double v : values_) {
        hash_combine(hashValue, v);
    }
    return hashValue;

}

void VectorLP::print(std::ostream &os) const {
    os << "(";
    for (std::vector<double>::const_iterator it = values_.begin();
            it < values_.end(); it++) {
        os << *it;
        if (it + 1 != values_.end()) {
            os << ",";
        }
    }
    os << ")";
}

std::unique_ptr<Point> VectorLP::copy() const {
    return std::make_unique<VectorLP>(values_, p_);
}

double VectorLP::distanceTo(Point const &otherPoint) const {
    VectorLP const &other = static_cast<VectorLP const &>(otherPoint);
    std::vector<double>::const_iterator i1 = values_.begin();
    std::vector<double>::const_iterator i2 = other.values_.begin();
    double pDistance = 0;
    for (; i1 < values_.end(); i1++, i2++) {
        pDistance += std::pow(std::abs(*i1 - *i2), p_);
    }
    return std::pow(pDistance, 1 / p_);
}

std::vector<double> VectorLP::asVector() const {
    return values_;
}

// Vector-like accessors follow.
double &VectorLP::operator[] (std::size_t index) {
    return values_[index];
}
double const &VectorLP::operator[](std::size_t index) const {
    return values_[index];
}

VectorLP::iterator VectorLP::begin() {
    return values_.begin();
}
VectorLP::const_iterator VectorLP::begin() const {
    return values_.begin();
}
VectorLP::const_iterator VectorLP::cbegin() const {
    return values_.cbegin();
}

VectorLP::iterator VectorLP::end() {
    return values_.end();
}
VectorLP::const_iterator VectorLP::end() const {
    return values_.end();
}
VectorLP::const_iterator VectorLP::cend() const {
    return values_.cend();
}
} /* namespace solver */
