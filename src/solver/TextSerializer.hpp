#ifndef SOLVER_TEXTSERIALIZER_HPP_
#define SOLVER_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for ostream, istream
#include <memory>                       // for unique_ptr
#include <queue>                        // for queue

#include "Observation.hpp"              // for Observation
#include "Serializer.hpp"               // for Serializer
#include "State.hpp"

namespace solver {
class ActionMapping;
class ActionNode;
class BeliefNode;
class BeliefTree;
class Histories;
class HistoryEntry;
class HistorySequence;
class ObservationMapping;
class Solver;
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

    virtual void save(std::vector<double> const &vector, std::ostream &os);
    virtual void load(std::vector<double> &vector, std::istream &is);

    virtual void saveState(State const *state, std::ostream &os);
    virtual std::unique_ptr<State> loadState(std::istream &is);

    virtual void saveObservation(Observation const *obs, std::ostream &os);
    virtual std::unique_ptr<Observation> loadObservation(std::istream &is);

    virtual void saveAction(Action const *action, std::ostream &os);
    virtual std::unique_ptr<Action> loadAction(std::istream &is);


    virtual void saveObservationMapping(ObservationMapping const &map,
            std::ostream &os);
    virtual std::unique_ptr<ObservationMapping> loadObservationMapping(
            std::istream &is);

    virtual void saveActionMapping(ActionMapping const &map,
            std::ostream &os);
    virtual std::unique_ptr<ActionMapping> loadActionMapping(
            std::istream &is);

    virtual void save(StateInfo const &wrapper, std::ostream &os);
    virtual void load(StateInfo &wrapper, std::istream &is);
    virtual void save(StatePool const &pool, std::ostream &os);
    virtual void load(StatePool &pool, std::istream &is);

    virtual void save(HistoryEntry const &entry, std::ostream &os);
    virtual void load(HistoryEntry &entry, std::istream &is);
    virtual void save(HistorySequence const &seq, std::ostream &os);
    virtual void load(HistorySequence &seq, std::istream &is);
    virtual void save(Histories const &histories, std::ostream &os);
    virtual void load(Histories &histories, std::istream &is);

    virtual void save(ActionNode const &node, std::ostream &os);
    virtual void load(ActionNode &node, std::istream &is);
    virtual void save(BeliefNode const &node, std::ostream &os);
    virtual void load(BeliefNode &node, std::istream &is);
    virtual void save(BeliefTree const &tree, std::ostream &os);
    virtual void load(BeliefTree &tree, std::istream &is);
};
} /* namespace solver */

#endif /* SOLVER_TEXTSERIALIZER_HPP_ */
