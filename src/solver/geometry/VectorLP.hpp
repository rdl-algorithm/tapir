#ifndef SOLVER_VECTORLP_HPP_
#define SOLVER_VECTORLP_HPP_

#include <initializer_list>
#include <memory>
#include <vector>

#include "global.hpp"
#include "Vector.hpp"

namespace solver {
class VectorLP : public Vector {
public:
    VectorLP(std::initializer_list<double> values, double p = 1.0);
    VectorLP(std::vector<double> values, double p = 1.0);
    virtual ~VectorLP() = default;
    _NO_COPY_OR_MOVE(VectorLP);

    virtual bool equals(Point const &otherPoint) const override;
    virtual std::size_t hash() const override;
    virtual void print(std::ostream &os) const override;

    virtual std::unique_ptr<Point> copy() const override;
    double distanceTo(Point const &otherPoint) const override;
    std::vector<double> asVector() const override;

    // Extra vector-like accessors for convenience.
    double &operator[](std::size_t index);
    double const &operator[](std::size_t index) const;

    typedef std::vector<double>::iterator iterator;
    typedef std::vector<double>::const_iterator const_iterator;
    iterator begin();
    const_iterator begin() const;
    const_iterator cbegin() const;

    iterator end();
    const_iterator end() const;
    const_iterator cend() const;
private:
    double p_;
    std::vector<double> values_;
};
} /* namespace solver */

#endif /* SOLVER_VECTORLP_HPP_ */
