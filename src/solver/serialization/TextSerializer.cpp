#include "TextSerializer.hpp"

#include <cstddef>                      // for size_t

#include <iostream>
#include <map>                          // for _Rb_tree_iterator, map<>::mapped_type
#include <sstream>                      // for stringstream
#include <string>                       // for operator>>, getline, string, operator<<
#include <unordered_set>                // for unordered_set<>::iterator, _Node_iterator
#include <utility>                      // for move, pair
#include <vector>                       // for vector

#include "solver/ActionNode.hpp"               // for ActionNode
#include "solver/BeliefNode.hpp"               // for BeliefNode, BeliefNode::ActionMap, BeliefNode::currId
#include "solver/BeliefTree.hpp"               // for BeliefTree
#include "solver/Histories.hpp"                // for Histories
#include "solver/HistoryEntry.hpp"             // for HistoryEntry
#include "solver/HistorySequence.hpp"          // for HistorySequence
#include "solver/Solver.hpp"                   // for Solver
#include "solver/StateInfo.hpp"                // for StateInfo, StateInfo::currId
#include "solver/StatePool.hpp"                // for StatePool, StatePool::StateInfoSet


#include "solver/mappings/ActionMapping.hpp"
#include "solver/mappings/ObservationMapping.hpp"          // for ObservationMapping

#include "solver/geometry/Action.hpp"              // for Observation
#include "solver/geometry/Observation.hpp"              // for Observation
#include "solver/geometry/State.hpp"                    // for State, operator<<
#include "solver/geometry/Vector.hpp"
#include "solver/geometry/VectorLP.hpp"

#include "Serializer.hpp"               // for Serializer

using std::endl;

namespace solver {
void TextSerializer::saveTransitionParameters(
        TransitionParameters const */*tp*/, std::ostream &/*os*/) {
}

std::unique_ptr<TransitionParameters> TextSerializer::loadTransitionParameters(
        std::istream &/*is*/) {
    return nullptr;
}

void TextSerializer::save(StateInfo const &info, std::ostream &os) {
    os << "State ";
    std::streamsize width = os.width(6);
    std::ios_base::fmtflags flags = os.setf(std::ios::right);
    os << info.id_;
    os.width(width);
    os.flags(flags);
    os << " : ";
    saveState(info.state_.get(), os);
}

void TextSerializer::load(StateInfo &info, std::istream &is) {
    std::string tmpStr;
    is >> tmpStr >> info.id_ >> tmpStr;
    info.state_ = loadState(is);
    if (info.id_ > StateInfo::currId) {
        StateInfo::currId = info.id_ + 1;
    }
}

void TextSerializer::save(StatePool const &pool, std::ostream &os) {
    os << "STATESPOOL-BEGIN" << endl;
    os << "#states: " << pool.stateInfoMap_.size() << endl;
    for (std::unique_ptr<StateInfo> const &stateInfo : pool.statesByIndex_) {
        save(*stateInfo, os);
        os << endl;
    }
    os << "STATESPOOL-END" << endl;
}

void TextSerializer::load(StatePool &pool, std::istream &is) {
    pool.reset();

    std::string line;
    std::getline(is, line);
    while (line.find("STATESPOOL-BEGIN") == std::string::npos) {
        std::ostringstream message;
        message << "WARNING: Junk line: ";
        message << line;
        debug::show_message(message.str());
        std::getline(is, line);
    }
    std::getline(is, line);
    std::string tmpStr;
    long nStates;
    std::istringstream(line) >> tmpStr >> nStates;

    StateInfo::currId = 0;

    std::getline(is, line);
    while (line.find("STATESPOOL-END") == std::string::npos) {
        std::unique_ptr<StateInfo> newStateInfo(std::make_unique<StateInfo>());
        std::istringstream sstr(line);
        load(*newStateInfo, sstr);
        pool.add(std::move(newStateInfo));
        std::getline(is, line);
    }
    StateInfo::currId = nStates;
}

void TextSerializer::save(HistoryEntry const &entry, std::ostream &os) {
    os << "HistoryEntry < ";
    std::streamsize width = os.width(5);
    os << entry.owningSequence_->id_;
    os.width(width);
    os << " " << entry.entryId_ << " >: (S ";
    os << entry.stateInfo_->id_;
    os << " ";
    saveAction(entry.action_.get(), os);
    os << " ";
    saveTransitionParameters(entry.transitionParameters_.get(), os);
    os << " ";
    saveObservation(entry.observation_.get(), os);
    os << " " << entry.discount_ << " " << entry.reward_ << " "
       << entry.totalDiscountedReward_ << " );";
    os << " S: ";
    saveState(entry.stateInfo_->getState(), os);
}

void TextSerializer::load(HistoryEntry &entry, std::istream &is) {
    std::string tmpStr;
    long stateId;
    long seqId;
    is >> tmpStr >> tmpStr >> seqId >> entry.entryId_;
    is >> tmpStr >> tmpStr >> stateId;
    entry.action_ = std::move(loadAction(is));
    entry.transitionParameters_ = std::move(loadTransitionParameters(is));
    entry.observation_ = std::move(loadObservation(is));
    is >> entry.discount_ >> entry.reward_;
    is >> entry.totalDiscountedReward_ >> tmpStr;
    entry.hasBeenBackedUp_ = true;
    entry.registerState(solver_->allStates_->getInfoById(stateId));
}

void TextSerializer::save(HistorySequence const &seq, std::ostream &os) {
    os << "HistorySequence " << seq.id_ << " - length " << seq.histSeq_.size()
       << " at depth " << seq.startDepth_ << endl;
    for (std::unique_ptr<HistoryEntry> const &entry : seq.histSeq_) {
        save(*entry, os);
        os << endl;
    }
}

void TextSerializer::load(HistorySequence &seq, std::istream &is) {
    long seqLength;
    std::string line;
    std::getline(is, line);
    std::istringstream sstr(line);
    std::string tmpStr;
    long id;
    sstr >> tmpStr >> id >> tmpStr >> tmpStr >> seqLength;
    sstr >> tmpStr >> tmpStr >> seq.startDepth_;
    for (int i = 0; i < seqLength; i++) {
        std::getline(is, line);
        std::unique_ptr<HistoryEntry> entry(std::make_unique<HistoryEntry>());
        sstr.clear();
        sstr.str(line);
        load(*entry, sstr);
        entry->owningSequence_ = &seq;
        seq.histSeq_.push_back(std::move(entry));
    }
}

void TextSerializer::save(Histories const &histories, std::ostream &os) {
    os << "HISTORIES-BEGIN" << endl;
    os << "#histories: " << histories.allHistSeq_.size() << endl;
    for (std::unique_ptr<HistorySequence> const &seq : histories.allHistSeq_) {
        save(*seq, os);
    }
    os << "HISTORIES-END" << endl;
}

void TextSerializer::load(Histories &histories, std::istream &is) {
    histories.reset();

    std::string line;
    std::getline(is, line);
    while (line.find("HISTORIES-BEGIN") == std::string::npos) {
        std::ostringstream message;
        message << "WARNING: Junk line: ";
        message << line;
        debug::show_message(message.str());
        std::getline(is, line);
    }

    long numHistories;
    std::string tmpStr;
    is >> tmpStr >> numHistories;
    std::getline(is, line);

    for (int i = 0; i < numHistories; i++) {
        std::unique_ptr<HistorySequence> sequence(std::make_unique<HistorySequence>());
        load(*sequence, is);
        histories.allHistSeq_.push_back(std::move(sequence));
    }
    std::getline(is, line);
    while (line.find("HISTORIES-END") == std::string::npos) {
        std::ostringstream message;
        message << "WARNING: Junk line: ";
        message << line;
        debug::show_message(message.str());
        std::getline(is, line);
    }
}

void TextSerializer::save(ActionNode const &node, std::ostream &os) {
    saveAction(node.action_.get(), os);
    os << " " << node.nParticles_ << " " << node.totalQValue_ << " ";
    os << node.meanQValue_ << " ";
    saveObservationMapping(*node.obsMap_, os);
}

void TextSerializer::load(ActionNode &node, std::istream &is) {
    std::string tmpStr;
    node.action_ = loadAction(is);
    is >> node.nParticles_ >> node.totalQValue_ >> node.meanQValue_;
    node.obsMap_ = std::move(loadObservationMapping(is));
}

void TextSerializer::save(BeliefNode const &node, std::ostream &os) {
    if (node.getNParticles() == 0) {
        os << "No particles!" << endl;
    } else {
        os << node.getNParticles() << " particles begin" << endl;
        int count = 0;
        for (auto it = node.particles_.begin(); it != node.particles_.end();
                ++it) {
            HistoryEntry const *entry = *it;
            os << "( ";
            std::streamsize width = os.width(5);
            os << entry->owningSequence_->id_;
            os.width(width);
            os << " " << entry->entryId_ << " )";
            count += 1;
            if (count >= NUM_PARTICLES_PER_LINE
                    || (it+1) == node.particles_.end()) {
                os << endl;
                count = 0;
            } else {
                os << ", ";
            }
        }
        os << "particles end" << endl;
    }
    saveActionMapping(*node.actionMap_, os);
}

void TextSerializer::load(BeliefNode &node, std::istream &is) {
    std::string line;
    std::getline(is, line);
    if (line != "No particles!") {
        long nParticles;

        std::istringstream(line) >> nParticles;

        std::getline(is, line);
        long numParticlesRead = 0;
        while (line != "particles end") {
            std::istringstream sstr(line);
            for (int i = 0; i < NUM_PARTICLES_PER_LINE; i++) {
                std::string tmpStr;
                long seqId, entryId;
                sstr >> tmpStr >> seqId >> entryId >> tmpStr;
                HistoryEntry *entry = solver_->allHistories_->getHistoryEntry(
                        seqId, entryId);
                entry->registerNode(&node);
                numParticlesRead++;
                if (numParticlesRead == nParticles) {
                    break;
                }
            }
            std::getline(is, line);
        }
    }
    node.actionMap_ = loadActionMapping(is);
}

void TextSerializer::save(BeliefTree const &tree, std::ostream &os) {
    os << "BELIEFTREE-BEGIN" << endl;
    os << "#nodes: " << BeliefNode::currId << endl;
    for (BeliefNode *node : tree.allNodes_) {
        os << "NODE # " << node->id_ << endl;
        save(*node, os);
        os << endl;
    }
    os << "BELIEFTREE-END" << endl;
}

void TextSerializer::load(BeliefTree &tree, std::istream &is) {
    std::string line;
    std::getline(is, line);
    while (line.find("BELIEFTREE-BEGIN") == std::string::npos) {
        std::ostringstream message;
        message << "WARNING: Junk line: ";
        message << line;
        debug::show_message(message.str());
        std::getline(is, line);
    }
    std::getline(is, line);

    long nNodes;
    std::string tmpStr;
    std::istringstream(line) >> tmpStr >> nNodes;
    tree.allNodes_.assign(nNodes, nullptr);
    tree.allNodes_[0] = tree.getRoot();
    tree.allNodes_[0]->id_ = 0;

    std::getline(is, line);
    while ((line.find("BELIEFTREE-END") == std::string::npos)) {
        long nodeId;
        std::istringstream(line) >> tmpStr >> tmpStr >> nodeId;
        BeliefNode *node = tree.allNodes_[nodeId];
        load(*node, is);
        if (node->getNActChildren() > 0) {
            node->updateBestValue();
        }
        // Ignore an empty line after each belief node.
        std::getline(is, line);
        std::getline(is, line);
    }
}
} /* namespace solver */
