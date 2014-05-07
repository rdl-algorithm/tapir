#include "TextSerializer.hpp"

#include <cstddef>                      // for size_t

#include <iomanip>
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

#include "solver/abstract-problem/Action.hpp"              // for Action
#include "solver/abstract-problem/ModelChange.hpp"              // for ModelChange
#include "solver/abstract-problem/Observation.hpp"              // for Observation
#include "solver/abstract-problem/State.hpp"                    // for State, operator<<

#include "Serializer.hpp"               // for Serializer

using std::endl;

namespace solver {
/* ------------------ Saving change sequences -------------------- */
void TextSerializer::saveChangeSequence(
        std::map<long, std::vector<std::unique_ptr<ModelChange>>> const &sequence,
        std::ostream &os) {
    for (auto &entry : sequence) {
        os << "t " << entry.first << " : " << entry.second.size() << std::endl;
        for (std::unique_ptr<ModelChange> const &change : entry.second) {
            saveModelChange(*change, os);
            os << std::endl;
        }
    }
}
std::map<long, std::vector<std::unique_ptr<ModelChange>>> TextSerializer::loadChangeSequence(
        std::istream &is){
    std::map<long, std::vector<std::unique_ptr<ModelChange>>> changes;
    std::string line;
    while (std::getline(is, line)) {
        std::string tmpStr;
        long time;
        long nChanges;
        std::istringstream(line) >> tmpStr >> time >> tmpStr >> nChanges;

        changes[time] = std::vector<std::unique_ptr<ModelChange>>();
        for (int i = 0; i < nChanges; i++) {
            std::getline(is, line);
            std::istringstream sstr(line);
            changes[time].push_back(loadModelChange(sstr));
        }
    }
    return changes;
}
void TextSerializer::saveModelChange(ModelChange const &/*change*/, std::ostream &/*os*/) {
    // Do nothing!
}
std::unique_ptr<ModelChange> TextSerializer::loadModelChange(std::istream &/*is*/) {
    return nullptr;
}

/* ------------------ Saving transition parameters -------------------- */
void TextSerializer::saveTransitionParameters(
        TransitionParameters const */*tp*/, std::ostream &/*os*/) {
    // Do nothing!
}
std::unique_ptr<TransitionParameters> TextSerializer::loadTransitionParameters(
        std::istream &/*is*/) {
    return nullptr;
}

/* ------------------ Saving historical data -------------------- */
void TextSerializer::saveHistoricalData(HistoricalData const */*data*/,
        std::ostream &/*os*/) {
}
std::unique_ptr<HistoricalData> TextSerializer::loadHistoricalData(
        std::istream &/*is*/) {
    return nullptr;
}

void TextSerializer::save(StateInfo const &info, std::ostream &os) {
    os << "State ";
    os << std::setw(6) << info.id_;
    os << " : ";
    saveState(info.state_.get(), os);
}

void TextSerializer::load(StateInfo &info, std::istream &is) {
    std::string tmpStr;
    is >> tmpStr >> info.id_ >> tmpStr;
    info.state_ = loadState(is);
}

void TextSerializer::save(StatePool const &pool, std::ostream &os) {
    os << "STATESPOOL-BEGIN" << endl;
    os << "numStates: " << pool.stateInfoMap_.size() << endl;
    for (std::unique_ptr<StateInfo> const &stateInfo : pool.statesByIndex_) {
        save(*stateInfo, os);
        os << endl;
    }
    os << "STATESPOOL-END" << endl;
}

void TextSerializer::load(StatePool &pool, std::istream &is) {
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

    std::getline(is, line);
    while (line.find("STATESPOOL-END") == std::string::npos) {
        std::unique_ptr<StateInfo> newStateInfo(std::make_unique<StateInfo>());
        std::istringstream sstr(line);
        load(*newStateInfo, sstr);
        pool.add(std::move(newStateInfo));
        std::getline(is, line);
    }
}

void TextSerializer::save(HistoryEntry const &entry, std::ostream &os) {
    os << "HistoryEntry < ";
    os << entry.owningSequence_->id_ << " " << entry.entryId_ << " >: (S";
    abt::print_with_width(entry.stateInfo_->id_, os, 6,
            std::ios_base::left);

    os << " ";
    std::stringstream sstr;
    saveAction(entry.action_.get(), sstr);
    abt::print_with_width(sstr.str(), os,
            getActionColumnWidth(),
            std::ios_base::left);

    os << " ";
    sstr.clear();
    sstr.str("");
    saveTransitionParameters(entry.transitionParameters_.get(), sstr);
    abt::print_with_width(sstr.str(), os,
            getTPColumnWidth(),
            std::ios_base::left);

    os << " ";
    sstr.clear();
    sstr.str("");
    saveObservation(entry.observation_.get(), sstr);
    abt::print_with_width(sstr.str(), os,
            getObservationColumnWidth(),
            std::ios_base::left);

    os << " r:";
    abt::print_double(entry.reward_, os, 6, 2,
            std::ios_base::fixed | std::ios_base::showpos
                    | std::ios_base::left);

    os << " v:";
    abt::print_double(entry.rewardFromHere_, os, 0, 1,
                std::ios_base::fixed | std::ios_base::showpos);

    os << ");   S:";
    saveState(entry.stateInfo_->getState(), os);
}

void TextSerializer::load(HistoryEntry &entry, std::istream &is) {
    std::string tmpStr;
    long stateId;
    long seqId;
    is >> tmpStr >> tmpStr >> seqId >> entry.entryId_;
    std::getline(is, tmpStr, 'S');
    is >> stateId;
    entry.action_ = std::move(loadAction(is));
    entry.transitionParameters_ = std::move(loadTransitionParameters(is));
    entry.observation_ = std::move(loadObservation(is));
    std::getline(is, tmpStr, ':');
    is >> entry.reward_;
    std::getline(is, tmpStr, ':');
    std::getline(is, tmpStr, ')');
    std::istringstream(tmpStr) >> entry.rewardFromHere_;
    entry.hasBeenBackedUp_ = true;
    entry.registerState(getSolver()->getStatePool()->getInfoById(stateId));
}

void TextSerializer::save(HistorySequence const &seq, std::ostream &os) {
    os << "HistorySequence " << seq.id_;
    os << " - length " << seq.entrySequence_.size() << std::endl;
    for (std::unique_ptr<HistoryEntry> const &entry : seq.entrySequence_) {
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
    sstr >> tmpStr >> seq.id_ >> tmpStr >> tmpStr >> seqLength;
    for (int i = 0; i < seqLength; i++) {
        std::getline(is, line);
        std::unique_ptr<HistoryEntry> entry(std::make_unique<HistoryEntry>());
        sstr.clear();
        sstr.str(line);
        load(*entry, sstr);
        entry->owningSequence_ = &seq;
        seq.entrySequence_.push_back(std::move(entry));
    }
}

void TextSerializer::save(Histories const &histories, std::ostream &os) {
    os << "HISTORIES-BEGIN" << endl;
    os << "numHistories: " << histories.getNumberOfSequences() << endl;
    for (std::unique_ptr<HistorySequence> const &seq : histories.sequencesById_) {
        save(*seq, os);
    }
    os << "HISTORIES-END" << endl;
}

void TextSerializer::load(Histories &histories, std::istream &is) {
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
        histories.sequencesById_.push_back(std::move(sequence));
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
    saveObservationMapping(*node.getMapping(), os);
}

void TextSerializer::load(ActionNode &node, std::istream &is) {
    node.setMapping(loadObservationMapping(is));
    for (ObservationMappingEntry const *entry : node.getMapping()->getAllEntries()) {
        BeliefNode *childNode = entry->getBeliefNode();
        if (childNode != nullptr) {
            getSolver()->getPolicy()->addNode(childNode);
        }
    }
}

void TextSerializer::save(BeliefNode const &node, std::ostream &os) {
    if (node.getNumberOfParticles() == 0) {
        os << "No particles!" << endl;
    } else {
        os << node.getNumberOfParticles() << " particles begin" << endl;
        int count = 0;
        for (auto it = node.particles_.begin(); it != node.particles_.end();
                ++it) {
            HistoryEntry const *entry = *it;
            os << "( ";
            os << entry->owningSequence_->id_;
            os << " ";
            os << entry->entryId_ << " )";
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
    saveHistoricalData(node.getHistoricalData(), os);
    saveActionMapping(*node.getMapping(), os);
}

void TextSerializer::load(BeliefNode &node, std::istream &is) {
    std::string line;
    std::getline(is, line);
    if (line != "No particles!") {
        long nParticles;
        std::string tmpStr;
        std::istringstream(line) >> nParticles;

        std::getline(is, line);
        long numParticlesRead = 0;
        while (line != "particles end") {
            std::istringstream sstr(line);
            for (int i = 0; i < NUM_PARTICLES_PER_LINE; i++) {
                long seqId, entryId;
                sstr >> tmpStr >> seqId >> entryId >> tmpStr;
                HistorySequence *sequence = getSolver()->histories_->getSequence(
                        seqId);
                HistoryEntry *entry = sequence->getEntry(entryId);
                entry->registerNode(&node);
                numParticlesRead++;
                if (numParticlesRead == nParticles) {
                    break;
                }
            }
            std::getline(is, line);
        }
    }
    node.setHistoricalData(loadHistoricalData(is));
    node.setMapping(loadActionMapping(is));
}

void TextSerializer::save(BeliefTree const &tree, std::ostream &os) {
    os << "BELIEFTREE-BEGIN" << endl;
    os << "numNodes: " << tree.getNumberOfNodes() << endl;
    for (BeliefNode *node : tree.allNodes_) {
        os << "NODE " << node->id_ << endl;
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
    tree.allNodes_.resize(nNodes);

    std::getline(is, line);
    while ((line.find("BELIEFTREE-END") == std::string::npos)) {
        long nodeId;
        std::istringstream(line) >> tmpStr >> nodeId;
        BeliefNode *node = tree.getNode(nodeId);
        load(*node, is);
        node->getMapping()->recalculate();
        // Ignore an empty line after each belief node.
        std::getline(is, line);
        std::getline(is, line);
    }
}

int TextSerializer::getActionColumnWidth(){
    return 0;
}
int TextSerializer::getTPColumnWidth() {
    return 0;
}
int TextSerializer::getObservationColumnWidth() {
    return 0;
}
} /* namespace solver */
