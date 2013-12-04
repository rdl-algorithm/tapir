#ifndef STATE_HPP
#define STATE_HPP

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream

class State {
public:
    State() = default;
    virtual ~State() = default;
    State(const State&) = delete;
    State(State&&) = delete;
    virtual State &operator=(const State&) = delete;
    virtual State &operator=(State&&) = delete;

    virtual double distanceTo(const State &otherState) const = 0;
    virtual bool equals(const State &otherState) const = 0;
    virtual std::size_t hash() const = 0;
    virtual void print(std::ostream &os) const = 0;

    friend std::ostream &operator<<(std::ostream &os, const State &state);
    friend bool operator==(const State &s1, const State &s2);
};

inline std::ostream &operator<<(std::ostream &os, const State &state) {
    state.print(os);
    return os;
}

inline bool operator==(const State &s1, const State &s2) {
    return s1.equals(s2); // && s2.equals(s1); (symmetry)
}

#endif /* STATE_HPP */
