#ifndef VECTORSTATE_HPP
#define VECTORSTATE_HPP

#include <cstddef>                      // for size_t

#include <ostream>                      // for operator<<, ostream, basic_ostream<>::__ostream_type
#include <unordered_map>                // for hash
#include <vector>                       // for vector, operator==, vector<>::iterator, vector<>::const_iterator

#include "solver/State.hpp"             // for State

using std::size_t;

/** Dodgy wrapped vector */
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

    virtual double distanceTo(State &otherState) const {
        VectorState *otherVectorState = static_cast<VectorState *>(&otherState);
        double distance = 0.0;
        typedef std::vector<double>::const_iterator DoubleIt;
        DoubleIt it1 = this->vals.cbegin();
        DoubleIt it2 = otherVectorState->vals.cbegin();
        for (; it1 != this->vals.cend(); it1++, it2++) {
            distance += std::abs(*it1 - *it2);
        }
        return distance;
    }

    virtual bool equals(State const &otherState) const {
        VectorState const *otherVectorState =
            static_cast<VectorState const *>(&otherState);
        return this->vals == otherVectorState->vals;
    }

    virtual size_t hash() const {
        size_t hashValue = 0;
        std::hash<double> hasher;
        for (double d : vals) {
            hashValue ^= hasher(d) + 0x9e3779b9
                + (hashValue<<6) + (hashValue>>2);
        }
        return hashValue;
    }

    virtual void print(std::ostream &os) const {
        for (double v : vals) {
            os << v << " ";
        }
    }







    double &operator[](size_t idx) {
        return vals[idx];
    }

    double const &operator[](size_t idx) const {
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

    std::vector<double> vals;
};

#endif /* VECTORSTATE_HPP */
