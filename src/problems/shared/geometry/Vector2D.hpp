#ifndef GEOMETRY_VECTOR2D_HPP_
#define GEOMETRY_VECTOR2D_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "solver/abstract-problem/Point.hpp"
#include "solver/abstract-problem/Vector.hpp"

#include "Point2D.hpp"

namespace geometry {

class Vector2D: public solver::Vector {
public:
    Vector2D();
    Vector2D(double magnitude, double direction);
    Vector2D(Point2D const &p);
    Vector2D(Point2D const &to, Point2D const &from);

    virtual ~Vector2D();

    virtual std::unique_ptr<solver::Point> copy() const override;
    virtual double distanceTo(Point const &otherPoint) const override;
    virtual bool equals(Point const &otherPoint) const override;
    virtual std::size_t hash() const override;

    virtual void print(std::ostream &os) const override;
    virtual void loadFrom(std::istream &is);

    virtual std::vector<double> asVector() const override;

    Point2D asPoint() const;
    double getX() const;
    double getY() const;
    double getMagnitude() const;
    double getDirection() const;

private:
    void calculateCartesian();
    void calculatePolar();

    double x_;
    double y_;
    double magnitude_;
    double direction_;
};

// operator<< is defined for Point, but operator>> isn't so we define it here.
inline std::istream &operator>>(std::istream &is, Vector2D &point) {
    point.loadFrom(is);
    return is;
}

inline Vector2D operator-(Point2D const &to, Point2D const &from) {
    return Vector2D(to, from);
}

inline Vector2D operator+(Vector2D const &v1, Vector2D const &v2) {
    return Vector2D(Point2D(v1.getX() + v2.getX(), v1.getY() + v2.getY()));
}

inline Vector2D operator-(Vector2D const &v1, Vector2D const &v2) {
    return Vector2D(Point2D(v1.getX() - v2.getX(), v1.getY() - v2.getY()));
}

inline Vector2D operator-(Vector2D const &v) {
    return Vector2D(Point2D(-v.getX(), -v.getY()));
}

inline Vector2D operator*(double scalar, Vector2D const &v) {
    return Vector2D(Point2D(scalar * v.getX(), scalar * v.getY()));
}

inline Vector2D operator*(Vector2D const &v, double scalar) {
	return scalar * v;
}

inline Point2D operator+(Point2D p, Vector2D const &v) {
    p += v;
    return p;
}

inline Point2D &operator+=(Point2D &p, Vector2D const &v) {
    p.x_ += v.getX();
    p.y_ += v.getY();
    return p;
}

inline Point2D operator+(Vector2D const &v, Point2D const &p) {
    return p + v;
}
} /* namespace geometry */

#endif /* GEOMETRY_VECTOR2D_HPP_ */
