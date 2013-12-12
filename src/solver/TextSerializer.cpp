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
#include "ObservationEdge.hpp"          // for ObservationEdge
#include "Serializer.hpp"               // for Serializer
#include "Solver.hpp"                   // for Solver
#include "State.hpp"                    // for State, operator<<
#include "StateInfo.hpp"                // for StateInfo, StateInfo::currId
#include "StatePool.hpp"                // for StatePool, StatePool::StateInfoSet

using std::cerr;
using std::endl;

namespace solver {
TextSerializer::TextSerializer() :
    TextSerializer(nullptr) {
}

TextSerializer::TextSerializer(Solver *solver) :
    Serializer(solver) {
}

void TextSerializer::save(StateInfo &info, std::ostream &os) {
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

void TextSerializer::save(StatePool &pool, std::ostream &os) {
    os << "STATESPOOL-BEGIN" << endl;
    os << "nStates: " << pool.allStates_.size() << endl;
    os << "nStVars: " << pool.nSDim_ << endl;
    for (StateInfo *stateInfo : pool.allStatesIdx_) {
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
    pool.allStatesIdx_.assign(nStates, nullptr);

    std::getline(is, line);
    while (line.find("STATESPOOL-END") == std::string::npos) {
        std::unique_ptr<StateInfo> newStateInfo(new StateInfo());
        sstr.clear();
        sstr.str(line);
        load(*newStateInfo, sstr);
        StateInfo *stateInfo = newStateInfo.get();
        typedef std::pair<StatePool::StateInfoSet::iterator, bool> ResultType;
        ResultType insertResult =
            pool.allStates_.insert(std::move(newStateInfo));
        if (!insertResult.second) {
            std::cerr << "Already inserted!?"
                      << *(*insertResult.first)->state_ << endl;
        }
        pool.allStatesIdx_[stateInfo->id_] = insertResult.first->get();
        for (long i = 0; i < pool.nSDim_; i++) {
//            pool.stStruct[i].insert(
//                    std::make_pair(newSt->state.vals[i],
//                            *(insertResult.first)));
        }
        std::getline(is, line);
    }
    StateInfo::currId = nStates;
}

void TextSerializer::save(Observation &obs, std::ostream &os) {
    os << obs.size() << " ";
    for (double v : obs) {
        os << v << " ";
    }
}

void TextSerializer::load(Observation &obs, std::istream &is) {
    obs.clear();
    std::size_t nStVars;
    is >> nStVars;
    for (std::size_t i = 0; i < nStVars; i++) {
        double tmpDouble;
        is >> tmpDouble;
        obs.push_back(tmpDouble);
    }
}

void TextSerializer::save(HistoryEntry &entry, std::ostream &os) {
    os << "HistoryEntry < " << entry.seqId_ << " " << entry.entryId_ << " >: ( "
       << entry.stateInfo_->id_ << " " << entry.action_ << " < ";
    save(entry.observation_, os);
    os << " > " << entry.discount_ << " " << entry.immediateReward_ << " "
       << entry.qVal_ << " ) ";
    save(*(entry.stateInfo_), os);
}

void TextSerializer::load(HistoryEntry &entry, std::istream &is) {
    std::string tmpStr;
    long stateId;
    is >> tmpStr >> tmpStr >> entry.seqId_ >> entry.entryId_ >> tmpStr >> tmpStr
    >> stateId >> entry.action_ >> tmpStr;
    load(entry.observation_, is);
    is >> tmpStr >> entry.discount_ >> entry.immediateReward_ >> entry.qVal_
    >> tmpStr;
    entry.hasBeenBackedUp_ = true;
    entry.stateInfo_ = solver_->allStates_->getStateById(stateId);
    entry.stateInfo_->addHistoryEntry(&entry);
}

void TextSerializer::save(HistorySequence &seq, std::ostream &os) {
    os << "HistorySequence " << seq.id_ << " - length " << seq.histSeq_.size()
       << endl;
    for (std::unique_ptr<HistoryEntry> &entry : seq.histSeq_) {
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
    sstr >> tmpStr >> id >> tmpStr >> tmpStr >> seqLength;
    for (int i = 0; i < seqLength; i++) {
        std::getline(is, line);
        sstr.clear();
        sstr.str(line);

        std::unique_ptr<HistoryEntry> entry(new HistoryEntry());
        load(*entry, sstr);
        seq.addEntry(std::move(entry));
    }
}

void TextSerializer::save(Histories &histories, std::ostream &os) {
    os << "HISTORIES-BEGIN" << endl;
    os << "numHistories: " << histories.allHistSeq_.size() << endl;
    for (std::unique_ptr<HistorySequence> &seq : histories.allHistSeq_) {
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
        std::unique_ptr<HistorySequence> sequence(new HistorySequence());
        load(*sequence, is);
        histories.allHistSeq_.push_back(std::move(sequence));
    }
    std::getline(is, line);
    while (line.find("HISTORIES-END") == std::string::npos) {
        std::cerr << "Junk line: " << line;
        std::getline(is, line);
    }
}

void TextSerializer::save(ObservationEdge &edge, std::ostream &os) {
    os << "O ( ";
    save(edge.observation_, os);
    os << " ) " << edge.child_->getId() << " ";
}

void TextSerializer::load(ObservationEdge &edge, std::istream &is) {
    std::string tmpStr;
    is >> tmpStr >> tmpStr;
    load(edge.observation_, is);
    long childId;
    is >> tmpStr >> childId;
    if (solver_->policy_->allNodes_[childId] != nullptr) {
        cerr << "Node already present !?" << endl;
    }
    edge.child_ = std::unique_ptr<BeliefNode>(new BeliefNode(childId));
    solver_->policy_->allNodes_[childId] = edge.getBeliefChild();
}

void TextSerializer::save(ActionNode &node, std::ostream &os) {
    os << "A " << node.action_ << " " << node.nParticles_ << " "
       << node.totalQValue_ << " " << node.meanQValue_ << " "
       << node.obsChildren.size() << " ";
    for (std::unique_ptr<ObservationEdge> &edge : node.obsChildren) {
        save(*edge, os);
    }
    os << endl;
}

void TextSerializer::load(ActionNode &node, std::istream &is) {
    std::string tmpStr;
    long nObs;
    is >> tmpStr >> node.action_ >> node.nParticles_ >> node.totalQValue_
    >> node.meanQValue_
    >> nObs;
    for (long i = 0; i < nObs; i++) {
        std::unique_ptr<ObservationEdge> edge(new ObservationEdge);
        load(*edge, is);
        node.obsChildren.push_back(std::move(edge));
    }
}

void TextSerializer::save(BeliefNode &node, std::ostream &os) {
    os << "Node " << node.id_ << endl;
    os << " " << node.nParticles_ << " " << node.getNActChildren() << " : ";
    for (HistoryEntry *entry : node.particles_) {
        os << "( " << entry->seqId_ << " " << entry->entryId_ << " ) ";
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
    long nActChildren;
    sstr >> node.nParticles_ >> nActChildren >> tmpStr;
    for (long i = 0; i < node.nParticles_; i++) {
        long seqId, entryId;
        sstr >> tmpStr >> seqId >> entryId >> tmpStr;
        HistoryEntry *entry = solver_->allHistories_->getHistoryEntry(seqId,
                    entryId);
        entry->owningBeliefNode_ = &node;
        node.particles_.push_back(entry);
    }
    for (long i = 0; i < nActChildren; i++) {
        std::getline(is, line);
        std::stringstream sstr2(line);
        std::unique_ptr<ActionNode> actionNode(new ActionNode());
        load(*actionNode, sstr2);
        node.actChildren_[actionNode->action_] = std::move(actionNode);
    }
}

void TextSerializer::save(BeliefTree &tree, std::ostream &os) {
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

    std::getline(is, line);
    while ((line.find("BELIEFTREE-END") == std::string::npos)) {
        sstr.clear();
        sstr.str(line);
        long nodeId;
        sstr >> tmpStr >> nodeId;
        BeliefNode *node = tree.allNodes_[nodeId];
        load(*node, is);
        node->updateBestValue();
        std::getline(is, line);
    }
}
} /* namespace solver */
