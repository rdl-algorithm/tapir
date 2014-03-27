#include "RockSampleMdpSolver.hpp"

#include <iostream>

#include "RockSampleModel.hpp"

namespace rocksample {
    RockSampleMdpSolver::RockSampleMdpSolver(RockSampleModel *model) :
            model_(model),
            valueMap_() {
        /*
        for (int i = 0; i < 1 << model_->getNRocks(); i++) {
            std::vector<bool> rockStates = model_->decodeRocks(i);
            std::unordered_set<int> goodRocks;
            for (int j = 0; j < model_->getNRocks(); j++) {
                if (rockStates[j]) {
                    goodRocks.insert(j);
                }
            }
            rockMap_.emplace(i, goodRocks);
        }
        */
    }

    void RockSampleMdpSolver::solve() {
        std::vector<std::pair<std::pair<int, int>, double>> entries;
        std::vector<std::pair<std::pair<int, int>, double>> newEntries;
        newEntries.push_back(std::make_pair(std::make_pair(0, -1), 0.0));
        for (int i = 0; i < model_->nRocks_; i++) {
            for (std::pair<std::pair<int, int>, double> const &entry : newEntries) {
                long rockCode = entry.first.first;
                long currentPos = entry.first.second;
                double value = entry.second;
                for (int j = 0; j < model_->nRocks_; j++) {
                    if ((rockCode & (1 << j)) == 0) {
                        long prevCode = rockCode | (1 << j);
                        long prevPos = model_->rockPositions_[]
                    }
                }
            }
        }
    }

    void RockSampleMdpSolver::save(std::ostream &os){

    }
    void RockSampleMdpSolver::load(std::istream &is) {

    }

} /* namespace rocksample */
