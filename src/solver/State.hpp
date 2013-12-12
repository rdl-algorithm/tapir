#ifndef SOLVER_STATE_HPP_
#define SOLVER_STATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream

namespace solver {
class State {
  public:
    struct Hash {
        size_t operator()(State const &state) const {
            return state.hash();
        }
    };
    State() = default;
    virtual ~State() = default;
    State(State const &) = delete;
    State(State &&) = delete;
    virtual State &operator=(State const &) = delete;
    virtual State &operator=(State &&) = delete;

    virtual double distanceTo(State const &otherState) const = 0;
    virtual bool equals(State const &otherState) const = 0;
    virtual std::size_t hash() const = 0;
    virtual void print(std::ostream &os) const = 0;

    friend std::ostream &operator<<(std::ostream &os, State const &state);
    friend bool operator==(State const &s1, State const &s2);
};

inline std::ostream &operator<<(std::ostream &os, State const &state) {
    state.print(os);
    return os;
}

inline bool operator==(State const &s1, State const &s2) {
    return s1.equals(s2); // && s2.equals(s1); (symmetry)
}
} /* namespace solver */

#endif /* SOLVER_STATE_HPP_ */
