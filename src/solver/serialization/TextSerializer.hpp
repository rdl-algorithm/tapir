#ifndef SOLVER_TEXTSERIALIZER_HPP_
#define SOLVER_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for ostream, istream
#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Observation.hpp"              // for Observation
#include "Serializer.hpp"               // for Serializer
#include "solver/abstract-problem/State.hpp"

#include "global.hpp"

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

/** Implements a text-based serialization of the core solver classes, i.e.
 * StatePool/StateInfo;
 * Histories/HistorySequence/HistoryEntry;
 * BeliefTree/BeliefNode/ActionNode;
 */
class TextSerializer : virtual public Serializer {
  public:
    TextSerializer() = default;
    virtual ~TextSerializer() = default;
    _NO_COPY_OR_MOVE(TextSerializer);

    virtual void saveTransitionParameters(TransitionParameters const *tp,
            std::ostream &os) override;
    virtual std::unique_ptr<TransitionParameters> loadTransitionParameters(
            std::istream &is) override;

    virtual void saveHistoricalData(HistoricalData const *data, std::ostream &os) override;
    virtual std::unique_ptr<HistoricalData> loadHistoricalData(std::istream &is) override;

    virtual void save(StateInfo const &wrapper, std::ostream &os) override;
    virtual void load(StateInfo &wrapper, std::istream &is) override;
    virtual void save(StatePool const &pool, std::ostream &os) override;
    virtual void load(StatePool &pool, std::istream &is) override;

    virtual void save(HistoryEntry const &entry, std::ostream &os) override;
    virtual void load(HistoryEntry &entry, std::istream &is) override;
    virtual void save(HistorySequence const &seq, std::ostream &os) override;
    virtual void load(HistorySequence &seq, std::istream &is) override;
    virtual void save(Histories const &histories, std::ostream &os) override;
    virtual void load(Histories &histories, std::istream &is) override;

    virtual void save(ActionNode const &node, std::ostream &os) override;
    virtual void load(ActionNode &node, std::istream &is) override;
    virtual void save(BeliefNode const &node, std::ostream &os) override;
    virtual void load(BeliefNode &node, std::istream &is) override;
    virtual void save(BeliefTree const &tree, std::ostream &os) override;
    virtual void load(BeliefTree &tree, std::istream &is) override;

    static const int NUM_PARTICLES_PER_LINE = 5;

    virtual int getActionColumnWidth();
    virtual int getTPColumnWidth();
    virtual int getObservationColumnWidth();
};
} /* namespace solver */

#endif /* SOLVER_TEXTSERIALIZER_HPP_ */
