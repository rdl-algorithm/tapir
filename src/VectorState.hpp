#ifndef VECTORSTATE_HPP
#define VECTORSTATE_HPP

#include <cstddef>                      // for size_t
#include <ostream>                      // for operator<<, basic_ostream<>::__ostream_type
#include <vector>                       // for vector, operator<, vector<>::iterator

#include "State.hpp"
using std::size_t;

class VectorState : public State {
public:
    VectorState() :
                vals() {
    }

    VectorState(size_t nVals) :
                vals(nVals) {
    }

    VectorState(std::vector<double> vals) :
                vals(vals) {
    }

    virtual ~VectorState() = default;
    std::vector<double> vals;

    double &operator[](size_t idx) {
        return vals[idx];
    }
    const double &operator[](size_t idx) const {
        return vals[idx];
    }
    std::vector<double>::iterator begin() {
        return vals.begin();
    }
    std::vector<double>::iterator end() {
        return vals.end();
    }
    void push_back(double v) {
        vals.push_back(v);
    }
    size_t size() {
        return vals.size();
    }
    void resize(size_t newSize) {
        vals.resize(newSize);
    }

    friend bool operator<(const VectorState &lhs, const VectorState &rhs);
    friend std::ostream &operator<<(std::ostream &os, const VectorState &state);
protected:
    virtual void print(std::ostream &os) const {
        for (double v : vals) {
            os << v << " ";
        }
    }
};

inline std::ostream &operator<<(std::ostream &os, const VectorState &state) {
    state.print(os);
    return os;
}

inline bool operator<(const VectorState &lhs, const VectorState &rhs) {
    return lhs.vals < rhs.vals;
}

#endif /* VECTORSTATE_HPP */
