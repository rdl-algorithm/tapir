#include "Rectangle2D.hpp"

#include <sstream>
#include <string>

namespace geometry {
Rectangle2D::Rectangle2D(Point2D lowerLeft, Point2D upperRight) :
        lowerLeft_(lowerLeft),
        upperRight_(upperRight) {
}

Rectangle2D::Rectangle2D(Rectangle2D const &other) :
        lowerLeft_(other.lowerLeft_),
        upperRight_(other.upperRight_) {
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
    std::getline(is, tmpStr, ')');
}

std::istream &operator>>(std::istream &is, Rectangle2D &rect) {
    rect.loadFrom(is);
    return is;
}

} /* namespace geometry */
