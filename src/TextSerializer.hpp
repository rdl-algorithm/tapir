#ifndef TEXTSERIALIZER_HPP
#define TEXTSERIALIZER_HPP

#include <iosfwd>

#include "Serializer.hpp"
class ActionNode;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationEdge;
class StatePool;
class StateWrapper;

class TextSerializer: public Serializer {
public:
    TextSerializer() = default;
    virtual ~TextSerializer() = default;

    virtual void save(StateWrapper &wrapper, std::ostream &os);
    virtual void load(StateWrapper *&wrapper, std::istream &is);
    virtual void save(StatePool &pool, std::ostream &os);
    virtual void load(StatePool *&pool, std::istream &is);

    virtual void save(HistoryEntry &entry, std::ostream &os);
    virtual void load(HistoryEntry *&entry, std::istream &is);
    virtual void save(HistorySequence &seq, std::ostream &os);
    virtual void load(HistorySequence *&seq, std::istream &is);
    virtual void save(Histories &histories, std::ostream &os);
    virtual void load(Histories *&histories, std::istream &is);

    virtual void save(ObservationEdge &edge, std::ostream &os);
    virtual void load(ObservationEdge *&edge, std::istream &is);
    virtual void save(ActionNode &node, std::ostream &os);
    virtual void load(ActionNode *&node, std::istream &is);
    virtual void save(BeliefNode &node, std::ostream &os);
    virtual void load(BeliefNode *&node, std::istream &is);
    virtual void save(BeliefTree &tree, std::ostream &os);
    virtual void load(BeliefTree *&tree, std::istream &is);
};

#endif /* TEXTSERIALIZER_HPP */
