#ifndef EUCLIDEANPOINT2D_HPP_
#define EUCLIDEANPOINT2D_HPP_

#include <cstddef>                      // for size_t

#include <memory>
#include <ostream>                      // for ostream
#include <vector>

#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "solver/State.hpp"
#include "solver/Vector.hpp"             // for State

class EuclideanPoint2D : public solver::Vector {
    friend class Nav2DTextSerializer;
  public:
    EuclideanPoint2D(double x, double y);

    virtual ~EuclideanPoint2D() = default;
    EuclideanPoint2D(EuclideanPoint2D const &);
    EuclideanPoint2D(EuclideanPoint2D &&) = delete;
    virtual EuclideanPoint2D &operator=(EuclideanPoint2D const &) = delete;
    virtual EuclideanPoint2D &operator=(EuclideanPoint2D &&) = delete;

    std::unique_ptr<solver::State> copy() const;

    double distanceTo(solver::State const &otherState) const;
    bool equals(solver::State const &otherState) const;
    std::size_t hash() const;

    std::vector<double> asVector() const;
    void print(std::ostream &os) const;

    double getX() const {
        return x_;
    }

    double getY() const {
        return y_;
    }

  private:
    double x_;
    double y_;
};

#endif /* EUCLIDEANPOINT2D_HPP_ */
