#ifndef SOLVER_TRANSITIONPARAMETERS_HPP_
#define SOLVER_TRANSITIONPARAMETERS_HPP_

namespace solver {

class TransitionParameters {
public:
    TransitionParameters() = default;
    virtual ~TransitionParameters() = default;

    virtual void print(std::ostream &os) const = 0;

    friend std::ostream &operator<<(std::ostream &os, Point const &point);
};

inline std::ostream &operator<<(std::ostream &os,
        TransitionParameters const &tp) {
    tp.print(os);
    return os;
}

} /* namespace solver */

#endif /* SOLVER_TRANSITIONPARAMETERS_HPP_ */
