#ifndef STATE_HPP
#define STATE_HPP

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream

class State {
  public:
    struct Hash {
        size_t operator()(State const *s1) const {
            return s1->hash();
        }
    };
    struct Same {
        bool operator()(State const *s1, State const *s2) const {
            return *s1 == *s2;
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

#endif /* STATE_HPP */
