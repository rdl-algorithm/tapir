#ifndef STATE_HPP
#define STATE_HPP

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream

class State {
public:
    virtual ~State();
    State(const State&) = delete;
    State(State&&) = delete;
    virtual State &operator=(const State&) = delete;
    virtual State &operator=(State&&) = delete;

    virtual double distL1(State &otherState) = 0;
    virtual size_t hash() = 0;

    virtual void print(std::ostream &os) const = 0;
    friend std::ostream &operator<<(std::ostream &os, const State &state);

    virtual bool equals(const State &otherState) const = 0;
    friend bool operator==(const State &s1, const State &s2);
};

inline std::ostream &operator<<(std::ostream &os, const State &state) {
    state.print(os);
    return os;
}

inline bool operator==(const State &s1, const State &s2) {
    return s1.equals(s2);
}

#endif /* STATE_HPP */
