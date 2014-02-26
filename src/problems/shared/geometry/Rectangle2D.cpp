#include "Rectangle2D.hpp"

#include "Point2D.hpp"

#include <sstream>
#include <string>

namespace geometry {
Rectangle2D::Rectangle2D() :
        Rectangle2D({0.0, 0.0}, {0.0, 0.0}) {
}

Rectangle2D::Rectangle2D(Point2D lowerLeft, Point2D upperRight) :
        lowerLeft_(lowerLeft),
        upperRight_(upperRight),
        area_(calculateArea()) {
}

Rectangle2D::Rectangle2D(Rectangle2D const &other) :
        Rectangle2D(other.lowerLeft_, other.upperRight_) {
}

Rectangle2D::~Rectangle2D() {
}

Rectangle2D &Rectangle2D::operator=(Rectangle2D const &other) {
    lowerLeft_ = other.lowerLeft_;
    upperRight_ = other.upperRight_;
    area_ = calculateArea();
    return *this;
}

void Rectangle2D::print(std::ostream &os) const {
    os << "(" << lowerLeft_ << ", " << upperRight_ << ")";
}

std::ostream &operator<<(std::ostream &os, Rectangle2D const &rect) {
    rect.print(os);
    return os;
}

void Rectangle2D::loadFrom(std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    is >> lowerLeft_;
    is >> upperRight_;
    area_ = calculateArea();
    std::getline(is, tmpStr, ')');
}

std::istream &operator>>(std::istream &is, Rectangle2D &rect) {
    rect.loadFrom(is);
    return is;
}

Point2D Rectangle2D::sampleUniform(RandomGenerator &randGen) const {
    double x = std::uniform_real_distribution<double>(lowerLeft_.getX(),
            upperRight_.getX())(randGen);
    double y = std::uniform_real_distribution<double>(lowerLeft_.getY(),
                upperRight_.getY())(randGen);
    return Point2D(x, y);
}

Point2D Rectangle2D::getLowerLeft() const {
    return lowerLeft_;
}

Point2D Rectangle2D::getUpperRight() const {
    return upperRight_;
}

double Rectangle2D::getArea() const {
    return area_;
}

bool Rectangle2D::contains(Point2D point) const {
    double x = point.getX();
    double y = point.getY();
    return (x >= lowerLeft_.getX() && x <= upperRight_.getX()
            && y >= lowerLeft_.getY() && y <= upperRight_.getY());

}

double Rectangle2D::calculateArea() const {
    return ((upperRight_.getX() - lowerLeft_.getX())
            * (upperRight_.getY() - lowerLeft_.getY()));
}
} /* namespace geometry */
