#ifndef SERIALIZER_HPP
#define SERIALIZER_HPP

#include <iosfwd>

class ActionNode;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationEdge;
class StatePool;
class StateWrapper;

class Serializer {
public:
    virtual ~Serializer() = default;

    virtual void save(StateWrapper &wrapper, std::ostream &os) = 0;
    virtual void load(StateWrapper *&wrapper, std::istream &is) = 0;
    virtual void save(StatePool &pool, std::ostream &os) = 0;
    virtual void load(StatePool *&pool, std::istream &is) = 0;

    virtual void save(HistoryEntry &entry, std::ostream &os) = 0;
    virtual void load(HistoryEntry *&entry, std::istream &is) = 0;
    virtual void save(HistorySequence &seq, std::ostream &os) = 0;
    virtual void load(HistorySequence *&seq, std::istream &is) = 0;
    virtual void save(Histories &histories, std::ostream &os) = 0;
    virtual void load(Histories *&histories, std::istream &is) = 0;

    virtual void save(ObservationEdge &edge, std::ostream &os) = 0;
    virtual void load(ObservationEdge *&edge, std::istream &is) = 0;
    virtual void save(ActionNode &node, std::ostream &os) = 0;
    virtual void load(ActionNode *&node, std::istream &is) = 0;
    virtual void save(BeliefNode &node, std::ostream &os) = 0;
    virtual void load(BeliefNode *&node, std::istream &is) = 0;
    virtual void save(BeliefTree &tree, std::ostream &os) = 0;
    virtual void load(BeliefTree *&tree, std::istream &is) = 0;
};

#endif /* SERIALIZER_HPP */
