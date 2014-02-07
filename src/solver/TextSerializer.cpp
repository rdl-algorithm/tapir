#include "TextSerializer.hpp"

#include <cstddef>                      // for size_t

#include <iostream>                     // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, basic_istream, endl, basic_istream<>::__istream_type, cerr, basic_ios::clear
#include <map>                          // for _Rb_tree_iterator, map<>::mapped_type
#include <sstream>                      // for stringstream
#include <string>                       // for operator>>, getline, string, operator<<
#include <unordered_set>                // for unordered_set<>::iterator, _Node_iterator
#include <utility>                      // for move, pair
#include <vector>                       // for vector

#include "ActionNode.hpp"               // for ActionNode
#include "BeliefNode.hpp"               // for BeliefNode, BeliefNode::ActionMap, BeliefNode::currId
#include "BeliefTree.hpp"               // for BeliefTree
#include "Histories.hpp"                // for Histories
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence
#include "Observation.hpp"              // for Observation
#include "ObservationMapping.hpp"          // for Ofor (double v : vector) {bservationMapping
#include "Serializer.hpp"               // for Serializer
#include "SimpleVectorL1.hpp"
#include "SimpleObsMap.hpp"          // for SimpleObsMap
#include "Solver.hpp"                   // for Solver
#include "State.hpp"                    // for State, operator<<
#include "StateInfo.hpp"                // for StateInfo, StateInfo::currId
#include "StatePool.hpp"                // for StatePool, StatePool::StateInfoSet
#include "Vector.hpp"

using std::cerr;
using std::endl;

namespace solver {
TextSerializer::TextSerializer() :
    TextSerializer(nullptr) {
}

TextSerializer::TextSerializer(Solver *solver) :
    Serializer(solver) {
}

void TextSerializer::save(std::vector<double> const &vector, std::ostream &os) {
    os << "<";
    for (std::vector<double>::const_iterator it = vector.begin();
            it != vector.end(); it++) {
        os << *it;
        if (it + 1 != vector.end()) {
            os << " ";
        }
    }
    os << ">";
}

void TextSerializer::load(std::vector<double> &vector, std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '<');
    std::getline(is, tmpStr, '>');

    vector.clear();

    if (!is.good() || tmpStr.empty()) {
        return;
    }

    std::stringstream sstr(tmpStr);
    while (sstr.good()) {
        double tmpDouble;
        sstr >> tmpDouble;
        vector.push_back(tmpDouble);
    }
}

void TextSerializer::saveState(State const &state, std::ostream &os) {
    std::vector<double> vector = ((Vector const &)state).asVector();
    save(vector, os);
}

std::unique_ptr<State> TextSerializer::loadState(std::istream &is) {
    std::vector<double> vector;
    load(vector, is);
    if (vector.empty()) {
        return nullptr;
    }
    return std::make_unique<SimpleVectorL1>(vector);
}

void TextSerializer::saveObservation(Observation const &obs, std::ostream &os) {
    std::vector<double> vector = ((Vector const &)obs).asVector();
    save(vector, os);
}

std::unique_ptr<State> TextSerializer::loadObservation(std::istream &is) {
    std::vector<double> vector;
    load(vector, is);
    if (vector.empty()) {
        return nullptr;
    }
    return std::make_unique<SimpleVectorL1>(vector);
}

void TextSerializer::save(StateInfo const &info, std::ostream &os) {
    os << "s " << info.id_ << " : ";
    saveState(*(info.state_), os);
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
    os << "nStates: " << pool.stateInfoMap_.size() << endl;
    os << "nStVars: " << pool.nSDim_ << endl;
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
        std::cerr << "Junk line: " << line;
        std::getline(is, line);
    }
    std::getline(is, line);
    std::stringstream sstr(line);
    std::string tmpStr;
    long nStates;
    sstr >> tmpStr >> nStates;
    sstr.clear();

    std::getline(is, line);
    sstr.str(line);
    sstr >> tmpStr >> pool.nSDim_;
    sstr.clear();

    StateInfo::currId = 0;

    std::getline(is, line);
    while (line.find("STATESPOOL-END") == std::string::npos) {
        std::unique_ptr<StateInfo> newStateInfo(std::make_unique<StateInfo>());
        sstr.clear();
        sstr.str(line);
        load(*newStateInfo, sstr);
        pool.add(std::move(newStateInfo));
        std::getline(is, line);
    }
    StateInfo::currId = nStates;
}

void TextSerializer::save(HistoryEntry const &entry, std::ostream &os) {
    (os << "HistoryEntry < " << entry.owningSequence_->id_ << " "
            << entry.entryId_ << " >: ( "
            << entry.stateInfo_->id_ << " " << entry.action_ << " ");
    if (entry.observation_ == nullptr) {
        os << "<>";
    } else {
        saveObservation(*entry.observation_, os);
    }
    os << " " << entry.discount_ << " " << entry.immediateReward_ << " "
       << entry.totalDiscountedReward_ << " ) ";
    save(*(entry.stateInfo_), os);
}

void TextSerializer::load(HistoryEntry &entry, std::istream &is) {
    std::string tmpStr;
    long stateId;
    long seqId;
    (is >> tmpStr >> tmpStr >> seqId >> entry.entryId_ >> tmpStr >> tmpStr
        >> stateId >> entry.action_);
    entry.observation_ = std::move(loadObservation(is));
    (is >> entry.discount_ >> entry.immediateReward_
        >> entry.totalDiscountedReward_ >> tmpStr);
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
    std::stringstream sstr(line);
    std::string tmpStr;
    long id;
    sstr >> tmpStr >> id >> tmpStr >> tmpStr >> seqLength
        >> tmpStr >> tmpStr >> seq.startDepth_;
    for (int i = 0; i < seqLength; i++) {
        std::getline(is, line);
        sstr.clear();
        sstr.str(line);

        std::unique_ptr<HistoryEntry> entry(std::make_unique<HistoryEntry>());
        load(*entry, sstr);
        entry->owningSequence_ = &seq;
        seq.histSeq_.push_back(std::move(entry));
    }
}

void TextSerializer::save(Histories const &histories, std::ostream &os) {
    os << "HISTORIES-BEGIN" << endl;
    os << "numHistories: " << histories.allHistSeq_.size() << endl;
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
        std::cerr << "Junk line: " << line;
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
        std::cerr << "Junk line: " << line;
        std::getline(is, line);
    }
}

void TextSerializer::saveMapping(ObservationMapping const &map, std::ostream &os) {
    os << "{";
    SimpleObsMap const &so = static_cast<SimpleObsMap const &>(map);
    if (so.obsNodes_.size() != 0) {
        os << " ";
        for (auto &entry : so.obsNodes_) {
            saveObservation(*entry.first, os);
            os << ":" << entry.second->id_ << " ";
        }
    }
    os << "}";
}

std::unique_ptr<ObservationMapping> TextSerializer::loadMapping(std::istream &is) {
    std::unique_ptr<SimpleObsMap> map(std::make_unique<SimpleObsMap>());
    std::string tmpStr;
    std::getline(is, tmpStr, '{');
    std::getline(is, tmpStr, '}');
    std::stringstream sstr(tmpStr);
    while (sstr.good()) {
        std::unique_ptr<Observation> obs = loadObservation(sstr);
        if (obs == nullptr) {
            break;
        }
        std::getline(sstr, tmpStr, ':');
        long childId;
        sstr >> childId;
        if (solver_->policy_->allNodes_[childId] != nullptr) {
            cerr << "ERROR: Node already present !?" << endl;
        }
        BeliefNode *node = map->createBelief(*obs);
        node->id_ = childId;
        solver_->policy_->allNodes_[childId] = node;
    }
    return std::move(map);
}

void TextSerializer::save(ActionNode const &node, std::ostream &os) {
    os << "A " << node.action_ << " " << node.nParticles_ << " "
       << node.totalQValue_ << " " << node.meanQValue_ << " ";
    saveMapping(*node.obsMap_, os);
    os << endl;
}

void TextSerializer::load(ActionNode &node, std::istream &is) {
    std::string tmpStr;
    is >> tmpStr >> node.action_ >> node.nParticles_ >> node.totalQValue_ >> node.meanQValue_;
    node.obsMap_ = loadMapping(is);
}

void TextSerializer::save(BeliefNode const &node, std::ostream &os) {
    os << "Node " << node.id_ << endl;
    os << " " << node.getNParticles() << " " << node.getNActChildren() << " : ";
    for (HistoryEntry *entry : node.particles_) {
        os << "( " << entry->owningSequence_->id_ << " " << entry->entryId_ << " ) ";
    }
    os << endl;
    for (auto actionNodeIter = node.actChildren_.begin();
         actionNodeIter != node.actChildren_.end(); actionNodeIter++) {
        save(*actionNodeIter->second, os);
    }
}

void TextSerializer::load(BeliefNode &node, std::istream &is) {
    std::string line;
    std::getline(is, line);
    std::stringstream sstr(line);
    std::string tmpStr;
    long nParticles;
    long nActChildren;
    sstr >> nParticles >> nActChildren >> tmpStr;
    for (long i = 0; i < nParticles; i++) {
        long seqId, entryId;
        sstr >> tmpStr >> seqId >> entryId >> tmpStr;
        HistoryEntry *entry = solver_->allHistories_->getHistoryEntry(seqId,
                    entryId);
        entry->registerNode(&node);
    }
    for (long i = 0; i < nActChildren; i++) {
        std::getline(is, line);
        std::stringstream sstr2(line);
        std::unique_ptr<ActionNode> actionNode(std::make_unique<ActionNode>());
        load(*actionNode, sstr2);
        node.actChildren_[actionNode->action_] = std::move(actionNode);
    }
}

void TextSerializer::save(BeliefTree const &tree, std::ostream &os) {
    os << "BELIEFTREE-BEGIN" << endl;
    os << "#nodes: " << BeliefNode::currId << endl;
    for (BeliefNode *node : tree.allNodes_) {
        save(*node, os);
    }
    os << "BELIEFTREE-END" << endl;
}

void TextSerializer::load(BeliefTree &tree, std::istream &is) {
    tree.reset();

    std::string line;
    std::getline(is, line);
    while (line.find("BELIEFTREE-BEGIN") == std::string::npos) {
        std::cerr << "Junk line: " << line;
        std::getline(is, line);
    }
    std::getline(is, line);

    long nNodes;
    std::stringstream sstr(line);
    std::string tmpStr;
    sstr >> tmpStr >> nNodes;
    tree.allNodes_.assign(nNodes, nullptr);
    tree.allNodes_[0] = tree.getRoot();
    tree.allNodes_[0]->id_ = 0;

    std::getline(is, line);
    while ((line.find("BELIEFTREE-END") == std::string::npos)) {
        sstr.clear();
        sstr.str(line);
        long nodeId;
        sstr >> tmpStr >> nodeId;
        BeliefNode *node = tree.allNodes_[nodeId];
        load(*node, is);
        if (node->getNActChildren() > 0) {
            node->updateBestValue();
        }
        std::getline(is, line);
    }
}
} /* namespace solver */
