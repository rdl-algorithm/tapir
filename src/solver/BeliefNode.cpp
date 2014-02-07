#include "BeliefNode.hpp"

#include <cmath>                        // for log, sqrt
#include <ctime>                        // for clock, CLOCKS_PER_SEC, clock_t

#include <iostream>                     // for cerr, endl, operator<<, basic_ostream, ostream
#include <map>                          // for _Rb_tree_iterator, map<>::iterator, map
#include <memory>                       // for unique_ptr
#include <random>                       // for uniform_int_distribution
#include <set>
#include <tuple>                        // for tie, tuple
#include <utility>                      // for pair, make_pair, move

#include "defs.hpp"                     // for RandomGenerator, make_unique

#warning Action should be a class!
#include "Action.hpp"                   // for Action
#include "ActionNode.hpp"               // for ActionNode
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "Observation.hpp"              // for Observation
#include "ParticleSet.hpp"
#include "State.hpp"                    // for State

using std::cerr;
using std::endl;

namespace solver {
long BeliefNode::currId = 0;
std::clock_t BeliefNode::startTime = std::clock();

BeliefNode::BeliefNode() :
    BeliefNode(currId) {
}

BeliefNode::BeliefNode(long id) :
    id_(id),
    nextActionToTry_(0),
    bestMeanQValue_(0),
    bestAction_(-1),
    tLastAddedParticle_(0),
    tNNComp_(-1.0),
    nnBel_(nullptr),
    particles_(),
    actChildren_() {
    if (currId <= id) {
        currId = id + 1;
    }
}

// Do-nothing destructor
BeliefNode::~BeliefNode() {
}

Action BeliefNode::getUcbAction(double ucbExploreCoefficient) {
    double tmpVal;
    ActionMap::iterator actionIter = actChildren_.begin();
    double maxVal = (actionIter->second->meanQValue_ + ucbExploreCoefficient
                     *std::sqrt(std::log(getNParticles())
                             / actionIter->second->nParticles_));
    Action bestActId = actionIter->first;
    actionIter++;
    for (; actionIter != actChildren_.end(); actionIter++) {
        tmpVal =
            (actionIter->second->meanQValue_ + ucbExploreCoefficient
             * std::sqrt(
                     std::log(getNParticles())
                     / actionIter->second->nParticles_));
        if (maxVal < tmpVal) {
            maxVal = tmpVal;
            bestActId = actionIter->first;
        }
    }
    return bestActId;
}

Action BeliefNode::getBestAction() const {
    return bestAction_;
}

double BeliefNode::getBestMeanQValue() const {
    return bestMeanQValue_;
}

void BeliefNode::updateBestValue() {
    if (getNActChildren() == 0) {
        bestAction_ = -1;
        bestMeanQValue_ = 0;
        cerr << "No children - could not update Q-value!" << endl;
        return;
    }
    ActionMap::iterator actionIter = actChildren_.begin();
    bestMeanQValue_ = actionIter->second->meanQValue_;
    bestAction_ = actionIter->first;
    actionIter++;
    for (; actionIter != actChildren_.end(); actionIter++) {
        if (bestMeanQValue_ < actionIter->second->meanQValue_) {
            bestMeanQValue_ = actionIter->second->meanQValue_;
            bestAction_ = actionIter->first;
        }
    }
}

void BeliefNode::addParticle(HistoryEntry *newHistEntry) {
    tLastAddedParticle_ = (double) (std::clock() - startTime)
        * 1000 / CLOCKS_PER_SEC;
    particles_.add(newHistEntry);
}

void BeliefNode::removeParticle(HistoryEntry *histEntry) {
    particles_.remove(histEntry);
}

HistoryEntry *BeliefNode::sampleAParticle(RandomGenerator *randGen) const {
    unsigned long index = std::uniform_int_distribution<unsigned long>(
                                 0, getNParticles() - 1)(*randGen);
    return particles_.get(index);
}

std::pair<BeliefNode *, bool> BeliefNode::createOrGetChild(Action const &action,
        Observation const &obs) {
    std::unique_ptr<ActionNode> newActionNode = std::make_unique<ActionNode>(
                action);
    bool added = false;
    std::map<Action, std::unique_ptr<ActionNode>>::iterator actionIter;
    std::tie(actionIter, added) = actChildren_.insert(std::make_pair(
                        action, std::move(newActionNode)));

    ActionNode *actChild = actionIter->second.get();
    return actChild->createOrGetChild(obs);
}

void BeliefNode::updateQValue(Action const &action, double increase) {
    updateQValue(action, increase, 0);
}

void BeliefNode::updateQValue(Action const &action, double increase,
        long deltaNParticles) {
    actChildren_[action]->updateQValue(increase, deltaNParticles);
    updateBestValue();
}

double BeliefNode::distL1Independent(BeliefNode *b) {
    double dist = 0.0;
    for (HistoryEntry *entry1 : particles_) {
        for (HistoryEntry *entry2 : particles_) {
            dist += entry1->getState()->distanceTo(*entry2->getState());
        }
    }
    return dist / (getNParticles() * b->getNParticles());
}

BeliefNode *BeliefNode::getChild(Action const &action, Observation const &obs) const {
    ActionMap::const_iterator iter = actChildren_.find(action);
    if (iter == actChildren_.end()) {
        return nullptr;
    }
    return iter->second->getChild(obs);
}

Action BeliefNode::getNextActionToTry() {
    return nextActionToTry_++;
}
} /* namespace solver */
