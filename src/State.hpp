#ifndef STATE_HPP
#define STATE_HPP

#include <ostream>
#include <vector>

class State {
public:
    State() :
                vals() {
    }
    State(std::vector<double> vals) :
                vals(vals) {
    }

    virtual ~State() = default;
    std::vector<double> vals;

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

#endif /* STATE_HPP */
