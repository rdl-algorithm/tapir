/** @file Rectangle2D.hpp
 *
 * Defines the Rectangle2D class, which represents an axis-aligned rectangle in 2-D Euclidean
 * space.
 */
#ifndef GEOMETRY_RECTANGLE2D_HPP_
#define GEOMETRY_RECTANGLE2D_HPP_

#include <iostream>

#include "global.hpp"

#include "problems/shared/geometry/Point2D.hpp"

namespace geometry {
/** A class modeling an axis-aligned rectangle in 2-D Euclidean space. */
class Rectangle2D {
public:
    /** Creates a new rectangle at (0, 0), with zero width and height. */
    Rectangle2D();
    /** Creates a new rectangle whose lower-left and upper-right corners are defined by the
     * two given points.
     */
    Rectangle2D(Point2D lowerLeft, Point2D upperRight);
    virtual ~Rectangle2D();

    /** Prints this rectangle out to the given output stream. */
    virtual void print(std::ostream &os) const;
    /** Loads the contents of this rectangle from the given input stream. */
    virtual void loadFrom(std::istream &is);

    /** Returns the distance from this rectangle to the given point. */
    double distanceTo(Point2D const &point) const;
    /** Returns the point within the bounds of this rectangle that is closest to the given point -
     * the returned point can be on the boundary of the rectangle.
     */
    Point2D closestPointTo(Point2D const &point) const;

    /** Samples a point at uniform from within the bounds of this rectangle, using the given random
     * engine.
     */
    Point2D sampleUniform(RandomGenerator &randGen) const;
    /** Returns the lower-left corner of this rectangle. */
    Point2D getLowerLeft() const;
    /** Returns the upper-right corner of this rectangle. */
    Point2D getUpperRight() const;
    /** Returns the area of this rectangle. */
    double getArea() const;
    /** Returns true if and only if the given point lies within the bounds of this rectangle;
     *  the boundary is included.
     */
    bool contains(Point2D point) const;

private:
    /** Calculates the area of this rectangle from its coordinates. */
    double calculateArea() const;
    /** The lower-left corner of this rectangle. */
    Point2D lowerLeft_;
    /** The upper-right corner of this rectangle. */
    Point2D upperRight_;
    /** The area of this rectangle. */
    double area_;
};

/** Outputs the contents of the given rectangle to the given output stream. */
inline std::ostream &operator<<(std::ostream &os, Rectangle2D const &rect) {
    rect.print(os);
    return os;
}

/** Loads the contents of the given rectangle from the given input stream. */
inline std::istream &operator>>(std::istream &is, Rectangle2D &rect) {
    rect.loadFrom(is);
    return is;
}

} /* namespace geometry */

#endif /* GEOMETRY_RECTANGLE2D_HPP_ */
