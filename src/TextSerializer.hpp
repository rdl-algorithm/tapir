#ifndef TEXTSERIALIZER_HPP
#define TEXTSERIALIZER_HPP

#include <iosfwd>                       // for ostream, istream
#include <queue>                        // for queue
#include <vector>                       // for vector
#include "Observation.hpp"              // for Observation
#include "Serializer.hpp"               // for Serializer
class ActionNode;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationEdge;
class Solver;
class State;
class StatePool;
class StateWrapper;

class TextSerializer: public Serializer {
  public:
    TextSerializer();
    TextSerializer(Solver *solver);
    virtual ~TextSerializer() = default;
    TextSerializer(TextSerializer const &) = delete;
    TextSerializer(TextSerializer &&) = delete;
    TextSerializer &operator=(TextSerializer const &) = delete;
    TextSerializer &operator=(TextSerializer &&) = delete;

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
    std::vector<BeliefNode *> nodeIndex;
    void saveWithChildren(ObservationEdge &edge, std::ostream &os,
                          std::queue<BeliefNode *> &queue);
    void saveWithChildren(ActionNode &node, std::ostream &os,
                          std::queue<BeliefNode *> &queue);
    void saveWithChildren(BeliefNode &node, std::ostream &os,
                          std::queue<BeliefNode *> &queue);
};

#endif /* TEXTSERIALIZER_HPP */
