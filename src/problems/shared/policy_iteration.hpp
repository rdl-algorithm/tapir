#ifndef POLICY_ITERATION_HPP_
#define POLICY_ITERATION_HPP_

#include "global.hpp"

#include <functional>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/SparseCore>
#include <eigen3/Eigen/SparseQR>

namespace mdp {
typedef int State;
typedef int Action;
typedef std::vector<Action> Policy;

class PolicyIterator {
public:
    PolicyIterator(Policy initialPolicy, double discountFactor, int numStates, int numActions,
            std::function<std::vector<State>(State, Action)> possibleNextStates,
            std::function<double(State, Action, State)> transitionProbability,
            std::function<double(State, Action, State)> reward);
    ~PolicyIterator() = default;
    _NO_COPY_OR_MOVE(PolicyIterator);

    void fixValue(State state, double value);
    void solve();
    Policy getCurrentPolicy();
    std::vector<double> getCurrentValues();

private:
    void updateValues();

    Policy policy_;
    std::vector<double> values_;
    std::unordered_map<int, double> fixedValues_;

    typedef Eigen::SparseMatrix<double, Eigen::ColMajor, int> SparseMatrixXd;
    typedef Eigen::SparseQR<SparseMatrixXd, Eigen::COLAMDOrdering<int>> SparseSolverXd;
    SparseMatrixXd coefficients_;
    Eigen::VectorXd constants_;

    double discountFactor_;
    int numStates_;
    int numActions_;
    std::function<std::vector<State>(State, Action)> possibleNextStates_;
    std::function<double(State, Action, State)> transitionProbability_;
    std::function<double(State, Action, State)> reward_;
};

} /* namespace mdp */

#endif /* POLICY_ITERATION_HPP_ */

