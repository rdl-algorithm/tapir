#include "Point2D.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "solver/geometry/Point.hpp"
#include "solver/geometry/Vector.hpp"

#include "global.hpp"

namespace geometry {
Point2D::Point2D() :
        Point2D(0.0, 0.0) {
}

Point2D::Point2D(double x, double y) :
    x_(x),
    y_(y) {
}

Point2D::Point2D(Point2D const &other) :
        Point2D(other.x_, other.y_) {
}

Point2D::Point2D(Point2D &&other) :
        Point2D(other.x_, other.y_) {
}

Point2D &Point2D::operator=(Point2D const &other) {
    x_ = other.x_;
    y_ = other.y_;
    return *this;
}

Point2D &Point2D::operator=(Point2D &&other) {
    x_ = other.x_;
    y_ = other.y_;
    return *this;
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
    abt::hash_combine(hashValue, x_);
    abt::hash_combine(hashValue, y_);
    return hashValue;
}

void Point2D::print(std::ostream& os) const {
    os << "(" << x_ << ", " << y_ << ")";
}

void Point2D::loadFrom(std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ',');
    std::istringstream iss(tmpStr);
    iss >> x_;
    std::getline(is, tmpStr, ')');
    iss.clear();
    iss.str(tmpStr);
    iss >> y_;
}

std::istream &operator>>(std::istream &is, Point2D &point) {
    point.loadFrom(is);
    return is;
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
