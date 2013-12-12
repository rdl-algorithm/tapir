#ifndef SOLVER_TEXTSERIALIZER_HPP_
#define SOLVER_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for ostream, istream
#include <memory>                       // for unique_ptr
#include <queue>                        // for queue

#include "Observation.hpp"              // for Observation
#include "Serializer.hpp"               // for Serializer

namespace solver {
class ActionNode;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationEdge;
class Solver;
class State;
class StateInfo;
class StatePool;

class TextSerializer : public Serializer {
  public:
    TextSerializer();
    TextSerializer(Solver *solver);
    virtual ~TextSerializer() = default;
    TextSerializer(TextSerializer const &) = delete;
    TextSerializer(TextSerializer &&) = delete;
    TextSerializer &operator=(TextSerializer const &) = delete;
    TextSerializer &operator=(TextSerializer &&) = delete;

    virtual void saveState(State &state, std::ostream &os) = 0;
    virtual std::unique_ptr<State> loadState(std::istream &is) = 0;

    virtual void save(StateInfo &wrapper, std::ostream &os);
    virtual void load(StateInfo &wrapper, std::istream &is);
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
    //std::vector<std::unique_ptr<BeliefNode> > nodeStore;
    void saveWithChildren(ObservationEdge &edge, std::ostream &os,
            std::queue<BeliefNode *> &queue);
    void saveWithChildren(ActionNode &node, std::ostream &os,
            std::queue<BeliefNode *> &queue);
    void saveWithChildren(BeliefNode &node, std::ostream &os,
            std::queue<BeliefNode *> &queue);
};
} /* namespace solver */

#endif /* SOLVER_TEXTSERIALIZER_HPP_ */
