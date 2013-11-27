#ifndef TEXTSERIALIZER_HPP
#define TEXTSERIALIZER_HPP

#include <iosfwd>

#include "Observation.hpp"
#include "Serializer.hpp"
#include "State.hpp"
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
    TextSerializer();
    TextSerializer(StatePool *statePool);
    virtual ~TextSerializer() = default;
    TextSerializer(const TextSerializer&) = delete;
    TextSerializer(TextSerializer&) = delete;
    TextSerializer &operator=(const TextSerializer&) = delete;
    TextSerializer &operator=(TextSerializer&) = delete;

    virtual void save(State &state, std::ostream &os);
    virtual void load(State &state, std::istream &is);
    virtual void save(StateWrapper &wrapper, std::ostream &os);
    virtual void load(StateWrapper &wrapper, std::istream &is);
    virtual void save(StatePool &pool, std::ostream &os);
    virtual void load(StatePool &pool, std::istream &is);

    virtual void save(Observation &obs, std::ostream &os);
    virtual void load(Observation &obs, std::istream &is);
    virtual void save(HistoryEntry &entry, std::ostream &os);
    virtual void load(HistoryEntry &entry, std::istream &is);
    virtual void save(HistorySequence &seq, std::ostream &os);
    virtual void load(HistorySequence &seq, std::istream &is);
    virtual void save(Histories &histories, std::ostream &os);
    virtual void load(Histories &histories, std::istream &is);

    virtual void save(ObservationEdge &edge, std::ostream &os);
    virtual void load(ObservationEdge &edge, std::istream &is);
    virtual void save(ActionNode &node, std::ostream &os);
    virtual void load(ActionNode &node, std::istream &is);
    virtual void save(BeliefNode &node, std::ostream &os);
    virtual void load(BeliefNode &node, std::istream &is);
    virtual void save(BeliefTree &tree, std::ostream &os);
    virtual void load(BeliefTree &tree, std::istream &is);
private:
    StatePool *statePool;
};

#endif /* TEXTSERIALIZER_HPP */
