#include "Nav2DObservation.hpp"

#include <cstddef>                      // for size_t
#include <cmath>                        // for pow

#include <functional>   // for hash
#include <ostream>                      // for operator<<, ostream, basic_ostream>
#include <vector>

#include "defs.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/State.hpp"             // for State

template<class T>
void hash_combine(std::size_t &seed, T const &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

EuclideanPoint2D::EuclideanPoint2D(double x, double y) :
    solver::Vector(),
    x_(x),
    y_(y) {
}

EuclideanPoint2D::EuclideanPoint2D(EuclideanPoint2D const &other) :
        EuclideanPoint2D(other.x_, other.y_) {
}

std::unique_ptr<solver::Point> EuclideanPoint2D::copy() const {
    return std::make_unique<EuclideanPoint2D>(this);
}

double EuclideanPoint2D::distanceTo(solver::State const &otherState) const {
    EuclideanPoint2D const &other = static_cast<EuclideanPoint2D const &>(
            otherState);
    return std::sqrt(std::pow(x_ - other.x_, 2) + std::pow(y_ - other.y_, 2));
}

bool EuclideanPoint2D::equals(solver::State const &otherState) const {
    EuclideanPoint2D const &other = static_cast<EuclideanPoint2D const &>(
            otherState);
    return (x_ == other.x_ && y_ == other.y_);
}

std::size_t EuclideanPoint2D::hash() const {
    std::size_t hashValue = 0;
    hash_combine(hashValue, x_);
    hash_combine(hashValue, y_);
    return hashValue;
}

std::vector<double> EuclideanPoint2D::asVector() const {
    return std::vector<double> {x_, y_};
}

void EuclideanPoint2D::print(std::ostream &os) const {
    os << "(" << x_ << ", " << y_ << ")";
}
