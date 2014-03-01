#ifndef GEOMETRY_VECTOR2D_HPP_
#define GEOMETRY_VECTOR2D_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "solver/geometry/Point.hpp"
#include "solver/geometry/Vector.hpp"

#include "Point2D.hpp"

namespace geometry {

class Vector2D: public solver::Vector {
public:
    Vector2D();
    Vector2D(double magnitude, double direction);
    Vector2D(Point2D const &p);
    Vector2D(Point2D const &to, Point2D const &from);

    virtual ~Vector2D();
    Vector2D(Vector2D const &other);
    Vector2D(Vector2D &&other);
    virtual Vector2D &operator=(Vector2D const &other);
    virtual Vector2D &operator=(Vector2D &&other);

    virtual std::unique_ptr<solver::Point> copy() const;
    virtual double distanceTo(Point const &otherPoint) const;
    virtual bool equals(Point const &otherPoint) const;
    virtual std::size_t hash() const;

    virtual void print(std::ostream &os) const;
    virtual void loadFrom(std::istream &is);

    virtual std::vector<double> asVector() const;

    Point2D asPoint() const;
    double getX() const;
    double getY() const;
    double getMagnitude() const;
    double getDirection() const;

    void calculateCartesian();
    void calculatePolar();

    friend std::istream &operator>>(std::istream &is, Vector2D &point);

    friend Point2D operator+(Point2D p, Vector2D const &v);
    friend Point2D &operator+=(Point2D &p, Vector2D const &v);
    friend Point2D operator+(Vector2D const &v, Point2D const &p);

    friend Vector2D operator+(Vector2D const &v1, Vector2D const &v2);
    friend Vector2D operator-(Vector2D const &v1, Vector2D const &v2);
    friend Vector2D operator-(Vector2D const &v);
    friend Vector2D operator*(double scalar, Vector2D const &v);
    friend Vector2D operator*(Vector2D const &v, double scalar);

private:
    double x_;
    double y_;
    double magnitude_;
    double direction_;
};

std::istream &operator>>(std::istream &is, Vector2D &point);

inline Vector2D operator-(Point2D const &to, Point2D const &from) {
    return Vector2D(to, from);
}

inline Vector2D operator+(Vector2D const &v1, Vector2D const &v2) {
    return Vector2D(Point2D(v1.x_ + v2.x_, v1.y_ + v2.y_));
}

inline Vector2D operator-(Vector2D const &v1, Vector2D const &v2) {
    return Vector2D(Point2D(v1.x_ - v2.x_, v1.y_ - v2.y_));
}

inline Vector2D operator-(Vector2D const &v) {
    return Vector2D(Point2D(-v.x_, -v.y_));
}

inline Vector2D operator*(double scalar, Vector2D const &v) {
    return Vector2D(Point2D(scalar * v.x_, scalar * v.y_));

}

inline Vector2D operator*(Vector2D const &v, double scalar) {
    return Vector2D(Point2D(scalar * v.x_, scalar * v.y_));
}

inline Point2D operator+(Point2D p, Vector2D const &v) {
    p += v;
    return p;
}

inline Point2D &operator+=(Point2D &p, Vector2D const &v) {
    p.x_ += v.x_;
    p.y_ += v.y_;
    return p;
}

inline Point2D operator+(Vector2D const &v, Point2D const &p) {
    return p + v;
}
} /* namespace geometry */

#endif /* GEOMETRY_VECTOR2D_HPP_ */
