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

using std::endl;

TextSerializer::TextSerializer() :
            TextSerializer(nullptr) {
}

TextSerializer::TextSerializer(Solver *solver) :
            Serializer(solver),
            nodeIndex() {
}

void TextSerializer::save(StateInfo &info, std::ostream &os) {
    os << "s " << info.id << " : ";
    save(*(info.state), os);
}

void TextSerializer::load(StateInfo &info, std::istream &is) {
    std::string tmpStr;
    is >> tmpStr >> info.id >> tmpStr;
    load(*(info.state), is);
    if (info.id > StateInfo::currId) {
        StateInfo::currId = info.id + 1;
    }
}

void TextSerializer::save(StatePool &pool, std::ostream &os) {
    os << "STATESPOOL-BEGIN" << endl;
    os << "nStates: " << pool.nStates << endl;
    os << "nStVars: " << pool.nSDim << endl;
    for (StateInfo *st : pool.allStates) {
        save(*st, os);
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
    sstr >> tmpStr >> pool.nStates;
    sstr.clear();

    std::getline(is, line);
    sstr.str(line);
    sstr >> tmpStr >> pool.nSDim;
    sstr.clear();
    pool.stStruct.resize(pool.nSDim);
    pool.allStatesIdx.assign(pool.nStates, nullptr);

    std::getline(is, line);
    while (line.find("STATESPOOL-END") == std::string::npos) {
        StateInfo *newSt = new StateInfo();
        std::stringstream sstr(line);
        load(*newSt, sstr);
        typedef std::pair<std::set<StateInfo*, CompStVals>::iterator, bool> ResultType;
        ResultType insertResult = pool.allStates.insert(newSt);
        pool.allStatesIdx[newSt->id] = *(insertResult.first);
        for (long i = 0; i < pool.nSDim; i++) {
            pool.stStruct[i].insert(
                    std::make_pair(newSt->state.vals[i],
                            *(insertResult.first)));
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
            << entry.stateInfo->getId() << " " << entry.actId << " < ";
    save(entry.obs, os);
    os << " > " << entry.discount << " " << entry.immediateReward << " " << entry.qVal << " ) ";
    save(*(entry.stateInfo), os);
}

void TextSerializer::load(HistoryEntry &entry, std::istream &is) {
    std::string tmpStr;
    long stateId;
    is >> tmpStr >> tmpStr >> entry.seqId >> entry.entryId >> tmpStr >> tmpStr
            >> stateId >> entry.actId >> tmpStr;
    load(entry.obs, is);
    is >> tmpStr >> entry.discount >> entry.immediateReward >> entry.qVal >> tmpStr;
    entry.hasBeenBackup = true;
    entry.stateInfo = solver->allStates->getStateById(stateId);
    entry.stateInfo->addHistoryEntry(&entry);
}

void TextSerializer::save(HistorySequence &seq, std::ostream &os) {
    os << "HistorySequence " << seq.id << " - length " << seq.histSeq.size()
            << endl;
    for (HistoryEntry *entry : seq.histSeq) {
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
        HistoryEntry *entry = new HistoryEntry();
        load(*entry, sstr);
        seq.addEntry(entry);
    }
}

void TextSerializer::save(Histories &histories, std::ostream &os) {
    os << "HISTORIES-BEGIN" << endl;
    os << "numHistories: " << histories.allHistSeq.size() << endl;
    for (HistorySequence *seq : histories.allHistSeq) {
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
        HistorySequence *sequence = new HistorySequence();
        load(*sequence, is);
        histories.allHistSeq.push_back(sequence);
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

void TextSerializer::saveWithChildren(ObservationEdge &edge, std::ostream &os,
        std::queue<BeliefNode *> &queue) {
    save(edge, os);
    edge.enqueueChildren(queue);
}

void TextSerializer::load(ObservationEdge &edge, std::istream &is) {
    std::string tmpStr;
    is >> tmpStr >> tmpStr;
    load(edge.obs, is);
    long childId;
    is >> tmpStr >> childId;
    edge.child = nodeIndex[childId];
    if (edge.child == nullptr) {
        edge.child = new BeliefNode(childId);
        nodeIndex[childId] = edge.child;
    }
}

void TextSerializer::save(ActionNode &node, std::ostream &os) {
    os << "A " << node.actId << " " << node.nParticles << " " << node.qVal
            << " " << node.avgQVal << " " << node.obsChildren.size() << " ";
}

void TextSerializer::saveWithChildren(ActionNode &node, std::ostream &os,
        std::queue<BeliefNode *> &queue) {
    save(node, os);
    for (ObservationEdge *edge : node.obsChildren) {
        saveWithChildren(*edge, os, queue);
    }
    os << endl;
}

void TextSerializer::load(ActionNode &node, std::istream &is) {
    std::string tmpStr;
    long nObs;
    is >> tmpStr >> node.actId >> node.nParticles >> node.qVal >> node.avgQVal
            >> nObs;
    for (long i = 0; i < nObs; i++) {
        ObservationEdge *edge = new ObservationEdge();
        load(*edge, is);
        node.addChild(edge);
    }
}

void TextSerializer::save(BeliefNode &node, std::ostream &os) {
    os << "Node " << node.id << endl;
    os << " " << node.nParticles << " " << node.nActChildren << " : ";
    for (HistoryEntry *entry : node.particles) {
        os << "( " << entry->getSeqId() << " " << entry->getId() << " ) ";
    }
    os << endl;
}

void TextSerializer::saveWithChildren(BeliefNode &node, std::ostream &os,
        std::queue<BeliefNode *> &queue) {
    save(node, os);
    for (std::pair<long, ActionNode *> actionChild : node.actChildren) {
        saveWithChildren(*(actionChild.second), os, queue);
    }
}

void TextSerializer::load(BeliefNode &node, std::istream &is) {
    std::string line;
    std::getline(is, line);
    std::stringstream sstr(line);
    std::string tmpStr;
    sstr >> node.nParticles >> node.nActChildren >> tmpStr;
    for (long i = 0; i < node.nParticles; i++) {
        long seqId, entryId;
        sstr >> tmpStr >> seqId >> entryId >> tmpStr;
        HistoryEntry *entry = solver->allHistories->getHistoryEntry(seqId,
                entryId);
        entry->setBelNode(&node);
        node.particles.push_back(entry);
    }
    for (long i = 0; i < node.nActChildren; i++) {
        std::getline(is, line);
        std::stringstream sstr(line);
        ActionNode *actionNode = new ActionNode();
        load(*actionNode, sstr);
        node.actChildren[actionNode->actId] = actionNode;
    }
}

void TextSerializer::save(BeliefTree &tree, std::ostream &os) {
    os << "BELIEFTREE-BEGIN" << endl;
    os << "#nodes: " << BeliefNode::currId << endl;
    std::queue<BeliefNode *> toBeWritten;
    toBeWritten.push(tree.root);
    while (!toBeWritten.empty()) {
        BeliefNode *node = toBeWritten.front();
        toBeWritten.pop();
        saveWithChildren(*node, os, toBeWritten);
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
    nodeIndex.assign(nNodes, nullptr);
    nodeIndex[0] = tree.root;

    std::getline(is, line);
    while ((line.find("BELIEFTREE-END") == std::string::npos)) {
        std::stringstream sstr(line);
        std::string tmpStr;
        long nodeId;
        sstr >> tmpStr >> nodeId;
        BeliefNode *node = nodeIndex[nodeId];
        load(*node, is);
        node->calcBestVal();
        std::getline(is, line);
    }
}
