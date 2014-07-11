/** @file RockSampleTextSerializer.hpp
 *
 * Contains text-based serialization methods for the core classes implementing RockSample, that is:
 * RockSampleChange, RockSampleState, RockSampleAction, and RockSampleObservation.
 */
#ifndef ROCKSAMPLE_TEXTSERIALIZER_HPP_
#define ROCKSAMPLE_TEXTSERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "solver/mappings/observations/enumerated_observations.hpp"

#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer

#include "position_history.hpp"
#include "smart_history.hpp"
#include "LegalActionsPool.hpp"
#include "global.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace rocksample {
/** A simple method to serialize a vector of longs to an output stream. */
void saveVector(std::vector<long> values, std::ostream &os);
/** A simple method to de-serialize a vector of longs from an input stream. */
std::vector<long> loadVector(std::istream &is);

/** A serialization class for the RockSample problem.
 *
 * This contains serialization methods for RockSampleChange, RockSampleState, RockSampleAction,
 * and RockSampleObservation.
 * this class also inherits from solver::EnumeratedObservationTextSerializer in order to serialize
 * the observation mappings.
 *
 * Note that this class does not implement any method to serialize the action mappings; this is
 * because different serialization is needed depending on what type of history-based heuristic
 * information is being used.
 */
class RockSampleTextSerializer: virtual public solver::TextSerializer,
    virtual public solver::EnumeratedObservationTextSerializer {
public:
    RockSampleTextSerializer() = default;
    /** Creates a new RockSampleTextSerializer instance, associated with the given solver. */
    RockSampleTextSerializer(solver::Solver *solver);
    virtual ~RockSampleTextSerializer() = default;
    _NO_COPY_OR_MOVE(RockSampleTextSerializer);

    /* ------------------ Saving change sequences -------------------- */
    virtual void saveModelChange(solver::ModelChange const &change, std::ostream &os) override;
    virtual std::unique_ptr<solver::ModelChange> loadModelChange(std::istream &is) override;

    void saveState(solver::State const *state, std::ostream &os) override;
    std::unique_ptr<solver::State> loadState(std::istream &is) override;

    void saveObservation(solver::Observation const *obs, std::ostream &os) override;
    std::unique_ptr<solver::Observation> loadObservation(std::istream &is) override;

    void saveAction(solver::Action const *action, std::ostream &os) override;
    std::unique_ptr<solver::Action> loadAction(std::istream &is) override;

    virtual int getActionColumnWidth() override;
    virtual int getTPColumnWidth() override;
    virtual int getObservationColumnWidth() override;
};

/** A serialization class for RockSample when the history-based data is disabled. */
class RockSampleBasicTextSerializer: public RockSampleTextSerializer,
    public solver::DiscretizedActionTextSerializer {
public:
    /** Creates a new instance for serialization without history-based data. */
    RockSampleBasicTextSerializer(solver::Solver *solver);
    virtual ~RockSampleBasicTextSerializer() = default;
};

/** A serialization class for RockSample when the history-based data is used to keep track of
 * legal actions.
 */
class RockSampleLegalActionsTextSerializer: public RockSampleTextSerializer,
        public LegalActionsPoolTextSerializer,
        public PositionDataTextSerializer {
public:
    /** Creates a new instance for serialization with mappings of legal actions. */
    RockSampleLegalActionsTextSerializer(solver::Solver *solver);
    virtual ~RockSampleLegalActionsTextSerializer() = default;
};

/** A serialization class for RockSample when the history-based data is used to determine
 * preferred actions, a la "smart knowledge" in POMCP.
 */
class RockSamplePreferredActionsTextSerializer: public RockSampleTextSerializer,
        public solver::DiscretizedActionTextSerializer,
        public PositionAndRockDataTextSerializer {
public:
    /** Creates a new instance for serialization with extra history-based inference. */
    RockSamplePreferredActionsTextSerializer(solver::Solver *solver);
    virtual ~RockSamplePreferredActionsTextSerializer() = default;
};

} /* namespace rocksample */

#endif /* ROCKSAMPLE_TEXTSERIALIZER_HPP_ */
