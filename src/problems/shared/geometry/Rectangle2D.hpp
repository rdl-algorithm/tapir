#ifndef GEOMETRY_RECTANGLE2D_HPP_
#define GEOMETRY_RECTANGLE2D_HPP_

#include <iostream>

#include "Point2D.hpp"

namespace geometry {
class Rectangle2D {
public:
    Rectangle2D(Point2D lowerLeft, Point2D upperRight);
    virtual ~Rectangle2D() = default;

    Rectangle2D(Rectangle2D const &other);
    Rectangle2D(Rectangle2D &&) = delete;
    virtual Rectangle2D &operator=(Rectangle2D const &) = delete;
    virtual Rectangle2D &operator=(Rectangle2D &&) = delete;

    virtual void print(std::ostream &os) const;
    virtual void loadFrom(std::istream &is);

    friend std::ostream &operator>>(std::istream &os, Rectangle2D const &rect);
private:
    Point2D lowerLeft_;
    Point2D upperRight_;
};

std::ostream &operator<<(std::ostream &os, Rectangle2D const &rect);
std::istream &operator>>(std::istream &is, Rectangle2D &rect);

} /* namespace geometry */

#endif /* GEOMETRY_RECTANGLE2D_HPP_ */
