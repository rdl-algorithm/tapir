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
    std::cout << std::endl;
    std::cout << "#states: " << numStates << std::endl;
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
            triplets.emplace_back(state, state, diagonalValue);
        }
    }

    coefficients_.setFromTriplets(triplets.begin(), triplets.end());

    // std::cout << coefficients_ << std::endl;
    SparseSolverXd solver;
//    std::cout << "Analyzing..." << std::endl;
    solver.compute(coefficients_);
    std::cout << (solver.info() == Eigen::Success) << std::endl;
//    std::cout << "Solving..." << std::endl;
    Eigen::VectorXd solution = solver.solve(constants_);
//    std::cout << "Solved..." << std::endl;
    for (State state = 0; state < numStates_; state++) {
        values_[state] = solution[state];
    }
}

void PolicyIterator::solve() {
    bool policyChanged = true;
    while (policyChanged) {
        policyChanged = false;
        updateValues();

        for (State state = 0; state < numStates_; state++) {
            double bestQ = values_[state];
            for (Action action = 0; action < numActions_; action++) {
                double actionQ = 0;
                for (State nextState : possibleNextStates_(state, action)) {
                    double prob = transitionProbability_(state, action, nextState);
                    actionQ += prob * (reward_(state, action, nextState)
                            + discountFactor_ * values_[nextState]);
                }
                if (actionQ > bestQ) {
                    policy_[state] = action;
                    bestQ = actionQ;
                    policyChanged = true;
                }
            }
        }
    }
}

Policy PolicyIterator::getCurrentPolicy() {
    return policy_;
}

std::vector<double> PolicyIterator::getCurrentValues() {
    return values_;
}
} /* namespace mdp */
