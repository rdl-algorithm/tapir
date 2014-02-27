#include "Vector2D.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Point2D.hpp"

#include "solver/geometry/Point.hpp"
#include "solver/geometry/Vector.hpp"

#include "utilities.hpp"
#include "global.hpp"

namespace geometry {
Vector2D::Vector2D() :
        Vector2D(Point2D(0.0, 0.0)) {
}

Vector2D::Vector2D(double magnitude, double direction) :
        x_(0),
        y_(0),
        magnitude_(magnitude),
        direction_(direction) {
    calculateCartesian();
}

Vector2D::Vector2D(Point2D const &p) :
            x_(p.getX()),
            y_(p.getY()),
            magnitude_(0),
            direction_(0) {
            calculatePolar();
}

Vector2D::Vector2D(Point2D const &to, Point2D const &from) :
        Vector2D(Point2D(to.getX() - from.getX(),
                to.getY() - from.getY())) {
}

Vector2D::Vector2D(Vector2D const &other) :
        Vector(),
        x_(other.x_),
        y_(other.y_),
        magnitude_(other.magnitude_),
        direction_(other.direction_) {
}

Vector2D::Vector2D(Vector2D &&other) :
        Vector(),
        x_(other.x_),
        y_(other.y_),
        magnitude_(other.magnitude_),
        direction_(
                other.direction_) {
}

Vector2D &Vector2D::operator=(Vector2D const &other) {
    x_ = other.x_;
    y_ = other.y_;
    magnitude_ = other.magnitude_;
    direction_ = other.direction_;
    return *this;
}

Vector2D &Vector2D::operator=(Vector2D &&other) {
    x_ = other.x_;
    y_ = other.y_;
    magnitude_ = other.magnitude_;
    direction_ = other.direction_;
    return *this;
}

Vector2D::~Vector2D() {
}

std::unique_ptr<solver::Point> Vector2D::copy() const {
    return std::make_unique<Vector2D>(*this);
}

double Vector2D::distanceTo(const Point& otherPoint) const {
    Vector2D const &other = static_cast<Vector2D const &>(otherPoint);
    return std::sqrt(std::pow(x_ - other.x_, 2) + std::pow(y_ - other.y_, 2));
}

bool Vector2D::equals(const Point& otherPoint) const {
    Vector2D const &other = static_cast<Vector2D const &>(otherPoint);
    return (x_ == other.x_ && y_ == other.y_);
}

std::size_t Vector2D::hash() const {
    std::size_t hashValue = 0;
    abt::hash_combine(hashValue, x_);
    abt::hash_combine(hashValue, y_);
    return hashValue;
}

void Vector2D::print(std::ostream& os) const {
    os << "(" << x_ << ", " << y_ << ")";
}

void Vector2D::loadFrom(std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ',');
    std::istringstream iss(tmpStr);
    iss >> x_;
    std::getline(is, tmpStr, ')');
    iss.clear();
    iss.str(tmpStr);
    iss >> y_;
    calculatePolar();
}

std::istream &operator>>(std::istream &is, Vector2D &point) {
    point.loadFrom(is);
    return is;
}

std::vector<double> Vector2D::asVector() const {
    return std::vector<double> {x_, y_};
}

Point2D Vector2D::asPoint() const {
    return Point2D(x_, y_);
}

double Vector2D::getX() const {
    return x_;
}

double Vector2D::getY() const {
    return y_;
}

double Vector2D::getMagnitude() const {
    return magnitude_;
}

double Vector2D::getDirection() const {
    return direction_;
}

void Vector2D::calculateCartesian() {
    x_ = magnitude_ * std::cos(2 * M_PI * direction_);
    y_ = magnitude_ * std::sin(2 * M_PI * direction_);
}

void Vector2D::calculatePolar() {
    magnitude_ = std::sqrt(std::pow(x_, 2) + std::pow(y_, 2));
    direction_ = normalizeTurn(std::atan2(y_, x_) /  (2 * M_PI));
    x_ = magnitude_ * std::cos(2 * M_PI * direction_);
    y_ = magnitude_ * std::sin(2 * M_PI * direction_);
}
} /* namespace geometry */
