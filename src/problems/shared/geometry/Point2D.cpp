/** @file Point2D.cpp
 *
 * Contains implementations for the methods of the Point2D class.
 */
#include "problems/shared/geometry/Point2D.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "global.hpp"

#include "solver/abstract-problem/Point.hpp"
#include "solver/abstract-problem/Vector.hpp"

#include "problems/shared/geometry/Vector2D.hpp"

namespace geometry {
Point2D::Point2D() :
        Point2D(0.0, 0.0) {
}

Point2D::Point2D(double x, double y) :
    x_(x),
    y_(y) {
}

Point2D::~Point2D() {
}

std::unique_ptr<solver::Point> Point2D::copy() const {
    return std::make_unique<Point2D>(*this);
}

double Point2D::distanceTo(const Point& otherPoint) const {
    Point2D const &other = static_cast<Point2D const &>(otherPoint);
    return std::sqrt(std::pow(x_ - other.x_, 2) + std::pow(y_ - other.y_, 2));
}

bool Point2D::equals(const Point& otherPoint) const {
    Point2D const &other = static_cast<Point2D const &>(otherPoint);
    return (x_ == other.x_ && y_ == other.y_);
}

std::size_t Point2D::hash() const {
    std::size_t hashValue = 0;
    tapir::hash_combine(hashValue, x_);
    tapir::hash_combine(hashValue, y_);
    return hashValue;
}

void Point2D::print(std::ostream& os) const {
    os << "(" << x_ << ", " << y_ << ")";
}

void Point2D::loadFrom(std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ',');
    std::istringstream(tmpStr) >> x_;
    std::getline(is, tmpStr, ')');
    std::istringstream(tmpStr) >> y_;
}

std::vector<double> Point2D::asVector() const {
    return std::vector<double> {x_, y_};
}

double Point2D::getX() const {
    return x_;
}

double Point2D::getY() const {
    return y_;
}
} /* namespace geometry */
