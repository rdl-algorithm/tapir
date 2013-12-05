#include "TextSerializer.hpp"

#include <cstddef>                      // for size_t

#include <iostream>                     // for ostream, istream, endl, cerr, etc.
#include <map>                          // for multimap, _Rb_tree_const_iterator, map
#include <queue>                        // for queue, __alloc_traits<>::value_type
#include <set>                          // for set<>::iterator, set
#include <sstream>                      // for stringstream
#include <string>                       // for operator>>, getline, string, operator<<
#include <utility>                      // for pair, make_pair
#include <vector>                       // for vector

#include "ActionNode.hpp"               // for ActionNode
#include "BeliefNode.hpp"               // for BeliefNode, BeliefNode::currId
#include "BeliefTree.hpp"               // for BeliefTree
#include "Histories.hpp"                // for Histories
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence
#include "Observation.hpp"              // for Observation
#include "ObservationEdge.hpp"          // for ObservationEdge
#include "Serializer.hpp"               // for Serializer
#include "Solver.hpp"                   // for Solver
#include "State.hpp"                    // for State
#include "StatePool.hpp"                // for StatePool, CompStVal
#include "StateInfo.hpp"             // for StateInfo, StateInfo::currId

using std::cerr;
using std::endl;

TextSerializer::TextSerializer() :
    TextSerializer(nullptr) {
}

TextSerializer::TextSerializer(Solver *solver) :
    Serializer(solver) {
}

void TextSerializer::save(StateInfo &info, std::ostream &os) {
    os << "s " << info.id << " : ";
    saveState(*(info.state), os);
}

void TextSerializer::load(StateInfo &info, std::istream &is) {
    std::string tmpStr;
    is >> tmpStr >> info.id >> tmpStr;
    info.state = loadState(is);
    if (info.id > StateInfo::currId) {
        StateInfo::currId = info.id + 1;
    }
}

void TextSerializer::save(StatePool &pool, std::ostream &os) {
    os << "STATESPOOL-BEGIN" << endl;
    os << "nStates: " << pool.allStates.size() << endl;
    os << "nStVars: " << pool.nSDim << endl;
    for (StateInfo *stateInfo : pool.allStatesIdx) {
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
    sstr >> tmpStr >> pool.nSDim;
    sstr.clear();
    pool.allStatesIdx.assign(nStates, nullptr);

    std::getline(is, line);
    while (line.find("STATESPOOL-END") == std::string::npos) {
        std::unique_ptr<StateInfo> newStateInfo(new StateInfo());
        std::stringstream sstr(line);
        load(*newStateInfo, sstr);
        typedef std::pair<StatePool::StateInfoSet::iterator, bool> ResultType;
        ResultType insertResult = pool.allStates.insert(std::move(newStateInfo));
        pool.allStatesIdx[newStateInfo->id] = insertResult.first->get();
        if (!insertResult.second) {
            std::cerr << "Already inserted!?" << *(*insertResult.first)->state << endl;
        }
        for (long i = 0; i < pool.nSDim; i++) {
//            pool.stStruct[i].insert(
//                    std::make_pair(newSt->state.vals[i],
//                            *(insertResult.first)));
        }
        std::getline(is, line);
    }
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
    os << "HistoryEntry < " << entry.seqId << " " << entry.entryId << " >: ( "
       << entry.stateInfo->getId() << " " << entry.action << " < ";
    save(entry.obs, os);
    os << " > " << entry.discount << " " << entry.immediateReward << " " << entry.qVal << " ) ";
    save(*(entry.stateInfo), os);
}

void TextSerializer::load(HistoryEntry &entry, std::istream &is) {
    std::string tmpStr;
    long stateId;
    is >> tmpStr >> tmpStr >> entry.seqId >> entry.entryId >> tmpStr >> tmpStr
       >> stateId >> entry.action >> tmpStr;
    load(entry.obs, is);
    is >> tmpStr >> entry.discount >> entry.immediateReward >> entry.qVal >> tmpStr;
    entry.hasBeenBackup = true;
    entry.stateInfo = solver->allStates->getStateById(stateId);
    entry.stateInfo->addHistoryEntry(&entry);
}

void TextSerializer::save(HistorySequence &seq, std::ostream &os) {
    os << "HistorySequence " << seq.id << " - length " << seq.histSeq.size()
       << endl;
    for (std::unique_ptr<HistoryEntry> &entry : seq.histSeq) {
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
        std::string line;
        std::getline(is, line);
        std::stringstream sstr(line);

        std::unique_ptr<HistoryEntry> entry(new HistoryEntry());
        load(*entry, sstr);
        seq.addEntry(std::move(entry));
    }
}

void TextSerializer::save(Histories &histories, std::ostream &os) {
    os << "HISTORIES-BEGIN" << endl;
    os << "numHistories: " << histories.allHistSeq.size() << endl;
    for (std::unique_ptr<HistorySequence> &seq : histories.allHistSeq) {
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
        histories.allHistSeq.push_back(std::move(sequence));
    }
    std::getline(is, line);
    while (line.find("HISTORIES-END") == std::string::npos) {
        std::cerr << "Junk line: " << line;
        std::getline(is, line);
    }
}

void TextSerializer::save(ObservationEdge &edge, std::ostream &os) {
    os << "O ( ";
    save(edge.obs, os);
    os << " ) " << edge.child->getId() << " ";
}

void TextSerializer::load(ObservationEdge &edge, std::istream &is) {
    std::string tmpStr;
    is >> tmpStr >> tmpStr;
    load(edge.obs, is);
    long childId;
    is >> tmpStr >> childId;
    if (solver->policy->allNodes[childId] != nullptr) {
        cerr << "Node already present !?" << endl;
    }
    edge.child = std::unique_ptr<BeliefNode>(new BeliefNode(childId));
    solver->policy->allNodes[childId] = edge.getBeliefChild();
}

void TextSerializer::save(ActionNode &node, std::ostream &os) {
    os << "A " << node.action << " " << node.nParticles << " "
            << node.totalQValue << " " << node.meanQValue << " "
            << node.obsChildren.size() << " ";
    for (std::unique_ptr<ObservationEdge> &edge : node.obsChildren) {
        save(*edge, os);
    }
    os << endl;
}

void TextSerializer::load(ActionNode &node, std::istream &is) {
    std::string tmpStr;
    long nObs;
    is >> tmpStr >> node.action >> node.nParticles >> node.totalQValue >> node.meanQValue
       >> nObs;
    for (long i = 0; i < nObs; i++) {
        std::unique_ptr<ObservationEdge> edge(new ObservationEdge);
        load(*edge, is);
        node.obsChildren.push_back(std::move(edge));
    }
}

void TextSerializer::save(BeliefNode &node, std::ostream &os) {
    os << "Node " << node.id << endl;
    os << " " << node.nParticles << " " << node.getNActChildren() << " : ";
    for (HistoryEntry *entry : node.particles) {
        os << "( " << entry->getSeqId() << " " << entry->getId() << " ) ";
    }
    os << endl;
    for (auto actionNodeIter = node.actChildren.begin();
            actionNodeIter != node.actChildren.end(); actionNodeIter++) {
        save(*actionNodeIter->second, os);
    }
}

void TextSerializer::load(BeliefNode &node, std::istream &is) {
    std::string line;
    std::getline(is, line);
    std::stringstream sstr(line);
    std::string tmpStr;
    long nActChildren;
    sstr >> node.nParticles >> nActChildren >> tmpStr;
    for (long i = 0; i < node.nParticles; i++) {
        long seqId, entryId;
        sstr >> tmpStr >> seqId >> entryId >> tmpStr;
        HistoryEntry *entry = solver->allHistories->getHistoryEntry(seqId,
                              entryId);
        entry->setBelNode(&node);
        node.particles.push_back(entry);
    }
    for (long i = 0; i < nActChildren; i++) {
        std::getline(is, line);
        std::stringstream sstr(line);
        std::unique_ptr<ActionNode> actionNode(new ActionNode());
        load(*actionNode, sstr);
        node.actChildren[actionNode->action] = std::move(actionNode);
    }
}

void TextSerializer::save(BeliefTree &tree, std::ostream &os) {
    os << "BELIEFTREE-BEGIN" << endl;
    os << "#nodes: " << BeliefNode::currId << endl;
    for (BeliefNode *node : tree.allNodes) {
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
    tree.allNodes.assign(nNodes, nullptr);
    tree.allNodes[0] = tree.getRoot();

    std::getline(is, line);
    while ((line.find("BELIEFTREE-END") == std::string::npos)) {
        std::stringstream sstr(line);
        std::string tmpStr;
        long nodeId;
        sstr >> tmpStr >> nodeId;
        BeliefNode *node = tree.allNodes[nodeId];
        load(*node, is);
        node->updateBestValue();
        std::getline(is, line);
    }
}
