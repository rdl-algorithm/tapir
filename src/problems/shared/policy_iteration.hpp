/** @file policy_iteration.hpp
 *
 * Contains the PolicyIterator class, which provides a basic implementation of MDP policy iteration;
 * the MDP solution can serve as a useful heuristic for many POMDPs.
 */
#ifndef POLICY_ITERATION_HPP_
#define POLICY_ITERATION_HPP_

#ifdef HAS_EIGEN

#include "global.hpp"

#include <functional>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Sparse>

/** A namespace to hold functionality for dealing with MDPs. */
namespace mdp {

/** States are assumed to be represented by numbers 0, 1, ... */
typedef int State;
/** Actions are assumed to be represented by numbers 0, 1, ... */
typedef int Action;

/** A policy is represented as a vector, where the state is an index into the vector, and the
 * action for that state is the element at that index.
 */
typedef std::vector<Action> Policy;

/** A class to handle basic MDP policy iteration. */
class PolicyIterator {
public:
    /** Creates a new PolicyIterator which will start with the given initial policy.
     *
     * The MDP is represented by the given discount factor and the given numbers of states and
     * actions; the functions possibleNextStates and transitionProbability represent the
     * conditional probability of transition to the next state given the state and action, while
     * the reward function encodes the reward for each transition.
     *
     * The solver simply calculates the value function at each step by solving the system of
     * equations V (I - gamma * T) = R, where R and T will depend on the policy for the current
     * step.
     */
    PolicyIterator(Policy initialPolicy, double discountFactor, int numStates, int numActions,
            std::function<std::vector<State>(State, Action)> possibleNextStates,
            std::function<double(State, Action, State)> transitionProbability,
            std::function<double(State, Action, State)> reward);
    ~PolicyIterator() = default;
    _NO_COPY_OR_MOVE(PolicyIterator);

    /** Sets the value of the given state as a fixed quantity - this is used to set reward values
     * for terminal states, if any exist.
     */
    void fixValue(State state, double value);

    /** Solves the MDP via policy iteration and returns the number of policy iteration steps taken
     * in attempting to solve the problem.
     */
    long solve();

    /** Returns the best policy calculated so far. */
    Policy getBestPolicy();

    /** Returns the value function via a vector of calculated values. */
    std::vector<double> getCurrentValues();

private:
    /** Updates the calculated value function for the current policy. */
    void updateValues();

    /** The best policy calculated so far. */
    Policy policy_;
    /** The current calculated value function. */
    std::vector<double> values_;
    /** The fixed values for this MDP, typically terminal states, or known values. */
    std::unordered_map<int, double> fixedValues_;

    /** The type of sparse matrix to use for the solution. */
    typedef Eigen::SparseMatrix<double, Eigen::ColMajor, int> SparseMatrixXd;
    /** The sparse solver to use in calculating the value function. */
    typedef Eigen::SparseLU<SparseMatrixXd, Eigen::COLAMDOrdering<int>> SparseSolverXd;

    /** The matrix of coefficients for transitions based on the current policy; represents
     * I - gamma * T
     */
    SparseMatrixXd coefficients_;
    /** The vector of reward values for each state, based on the current policy. */
    Eigen::VectorXd constants_;

    /** The MDP discount factor. */
    double discountFactor_;
    /** The number of states in the MDP. */
    int numStates_;
    /** The number of actions in the MDP. */
    int numActions_;
    /** The function returning the possible next states for each (state, action) pair. */
    std::function<std::vector<State>(State, Action)> possibleNextStates_;
    /** The function returning the conditional probability for transition, P(s' | s, a). */
    std::function<double(State, Action, State)> transitionProbability_;
    /** The function returning the reward, R(s, a, s'). */
    std::function<double(State, Action, State)> reward_;
};

} /* namespace mdp */

#endif /* HAS_EIGEN */
#endif /* POLICY_ITERATION_HPP_ */
