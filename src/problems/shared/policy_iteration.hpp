#ifndef POLICY_ITERATION_HPP_
#define POLICY_ITERATION_HPP_

#include <functional>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace mdp {
typedef int State;
typedef int Action;
typedef std::vector<Action> Policy;

class PolicyIterator {
public:
    /** Solves an MDP using policy iteration. */
    std::vector<double> solve(Policy &policy, double discountFactor, int numStates, int numActions,
            std::function<std::vector<State>(State, Action)> possibleNextStates,
            std::function<double(State, Action, State)> transitionProbability,
            std::function<double(State, Action, State)> reward);

    /** Solves for the values given the current policy. */
    std::vector<double> getValues();
};

} /* namespace solve */

#endif /* POLICY_ITERATION_HPP_ */
