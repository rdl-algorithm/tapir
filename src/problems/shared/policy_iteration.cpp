#include "policy_iteration.hpp"

#include <iostream>

namespace mdp {
PolicyIterator::PolicyIterator(Policy initialPolicy, double discountFactor, int numStates,
        int numActions, std::function<std::vector<State>(State, Action)> possibleNextStates,
        std::function<double(State, Action, State)> transitionProbability,
        std::function<double(State, Action, State)> reward) :
        policy_(initialPolicy),
        values_(numStates),
        fixedValues_(),
        coefficients_(numStates, numStates),
        constants_(numStates),
        discountFactor_(discountFactor),
        numStates_(numStates),
        numActions_(numActions),
        possibleNextStates_(possibleNextStates),
        transitionProbability_(transitionProbability),
        reward_(reward) {
}

void PolicyIterator::fixValue(State state, double value) {
    fixedValues_[state] = value;
}

void PolicyIterator::updateValues() {
    coefficients_.setZero();
    constants_.setZero();

    std::vector<Eigen::Triplet<double>> triplets;

    for (State state = 0; state < numStates_; state++) {
        auto it = fixedValues_.find(state);
        double diagonalValue = 1.0;
        if (it != fixedValues_.end()) {
            constants_[state] = it->second;
        } else {
            Action action = policy_[state];
            for (State nextState : possibleNextStates_(state, action)) {
                double prob = transitionProbability_(state, action, nextState);
                double value = -discountFactor_ * prob;
                if (state == nextState) {
                    diagonalValue += value;
                } else {
                    triplets.emplace_back(state, nextState, value);
                }
                constants_[state] += prob * reward_(state, action, nextState);
            }
        }
        triplets.emplace_back(state, state, diagonalValue);
    }
    coefficients_.setFromTriplets(triplets.begin(), triplets.end());
    SparseSolverXd solver;
    solver.compute(coefficients_);
    Eigen::VectorXd solution = solver.solve(constants_);

    for (State state = 0; state < numStates_; state++) {
        values_[state] = solution[state];
    }
}

long PolicyIterator::solve() {
    bool policyChanged = true;

    long numIterations = 0;
    while (policyChanged) {
        numIterations++;
        policyChanged = false;
        updateValues();

        for (State state = 0; state < numStates_; state++) {
            // Ignore terminal states.
            if (fixedValues_.count(state) > 0) {
                continue;
            }

            double bestQ = values_[state];
            for (Action action = 0; action < numActions_; action++) {
                // Ignore the current best action.
                if (action == policy_[state]) {
                    continue;
                }

                double actionQ = 0;
                for (State nextState : possibleNextStates_(state, action)) {
                    double prob = transitionProbability_(state, action, nextState);
                    actionQ += prob * (reward_(state, action, nextState)
                            + discountFactor_ * values_[nextState]);
                }
                if (actionQ > bestQ + 1e-15) {
                    policy_[state] = action;
                    bestQ = actionQ;
                    policyChanged = true;
                }
            }
        }
    }
    return numIterations;
}

Policy PolicyIterator::getCurrentPolicy() {
    return policy_;
}

std::vector<double> PolicyIterator::getCurrentValues() {
    return values_;
}
} /* namespace mdp */
