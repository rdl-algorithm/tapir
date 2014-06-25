#include "policy_iteration.hpp"

#include <eigen3/Eigen/Dense>

namespace mdp {
std::vector<double> solve(Policy &policy, double discountFactor,
        int numStates, int numActions,
        std::function<std::vector<State>(State, Action)> possibleNextStates,
        std::function<double(State, Action, State)> transitionProbability,
        std::function<double(State, Action, State)> reward) {

    std::vector<double> values;
    values.resize(numStates);
    bool policyChanged = true;
    while (policyChanged) {
        policyChanged = false;
        values = getValues();

        for (int stateNo = 0; stateNo < numStates; stateNo++) {
            double bestQ = values[stateNo];
        }
    }

    return values;
}
} /* namespace mdp */
