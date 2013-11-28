#ifndef STATE_HPP
#define STATE_HPP

#include <ostream>
#include <vector>

class State {
public:
    State() :
                vals() {
    }

    State(std::size_t nVals) :
                vals(nVals) {
    }

    State(std::vector<double> vals) :
                vals(vals) {
    }

    virtual ~State() = default;
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

    friend bool operator<(const State &lhs, const State &rhs);
    friend std::ostream &operator<<(std::ostream &os, const State &state);
protected:
    virtual void print(std::ostream &os) const {
        for (double v : vals) {
            os << v << " ";
        }
    }
};

inline std::ostream &operator<<(std::ostream &os, const State &state) {
    state.print(os);
    return os;
}

inline bool operator<(const State &lhs, const State &rhs) {
    return lhs.vals < rhs.vals;
}

#endif /* STATE_HPP */
