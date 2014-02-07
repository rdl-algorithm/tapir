#ifndef SOLVER_SIMPLEVECTORL1_HPP_
#define SOLVER_SIMPLEVECTORL1_HPP_

#include <memory>
#include <vector>

#include "Vector.hpp"

namespace solver {
class SimpleVectorL1 : public Vector {
public:
    SimpleVectorL1(std::vector<double> values);
    virtual ~SimpleVectorL1() = default;

    SimpleVectorL1(SimpleVectorL1 const &) = delete;
    SimpleVectorL1(SimpleVectorL1 &&) = delete;
    virtual SimpleVectorL1 &operator=(SimpleVectorL1 const &) = delete;
    virtual SimpleVectorL1 &operator=(SimpleVectorL1 &&) = delete;

    virtual std::unique_ptr<Point> copy() const;

    double distanceTo(Point const &otherPoint) const;

    virtual std::vector<double> asVector() const;
private:
    std::vector<double> values_;
};

} /* namespace solver */

#endif /* SOLVER_SIMPLEVECTORL1_HPP_ */
