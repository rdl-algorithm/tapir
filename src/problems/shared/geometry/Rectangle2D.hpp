#ifndef GEOMETRY_RECTANGLE2D_HPP_
#define GEOMETRY_RECTANGLE2D_HPP_

#include <iostream>

#include "Point2D.hpp"
#include "global.hpp"

namespace geometry {
class Rectangle2D {
public:
    Rectangle2D();
    Rectangle2D(Point2D lowerLeft, Point2D upperRight);
    virtual ~Rectangle2D();

    Rectangle2D(Rectangle2D const &other);
    Rectangle2D(Rectangle2D &&) = delete;
    virtual Rectangle2D &operator=(Rectangle2D const &);
    virtual Rectangle2D &operator=(Rectangle2D &&) = delete;

    virtual void print(std::ostream &os) const;
    virtual void loadFrom(std::istream &is);

    double distanceTo(Point2D const &point) const;
    Point2D closestPointTo(Point2D const &point) const;

    Point2D sampleUniform(RandomGenerator &randGen) const;
    Point2D getLowerLeft() const;
    Point2D getUpperRight() const;
    double getArea() const;
    bool contains(Point2D point) const;

    friend std::ostream &operator>>(std::istream &os, Rectangle2D const &rect);
private:
    double calculateArea() const;
    Point2D lowerLeft_;
    Point2D upperRight_;
    double area_;
};

std::ostream &operator<<(std::ostream &os, Rectangle2D const &rect);
std::istream &operator>>(std::istream &is, Rectangle2D &rect);

} /* namespace geometry */

#endif /* GEOMETRY_RECTANGLE2D_HPP_ */
