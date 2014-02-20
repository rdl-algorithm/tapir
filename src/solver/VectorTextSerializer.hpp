#ifndef SOLVER_VECTORTEXTSERIALIZER_HPP_
#define SOLVER_VECTORTEXTSERIALIZER_HPP_

#include "Serializer.hpp"

#include "global.hpp"

namespace solver {
/** Simple vector-based serialization of States, Observations, and Actions;
 * using an L1 metric where needed.
 * If you use a custom subclass of State/Observation/Action you will need
 * to override the serialization for that subclass.
 */
class VectorTextSerializer : virtual public Serializer {
  public:
    VectorTextSerializer() = default;
    virtual ~VectorTextSerializer() = default;
    _NO_COPY_OR_MOVE(VectorTextSerializer);

    /** Saves a vector to an output stream. */
    virtual void save(std::vector<double> const &vector, std::ostream &os);
    /** Loads a vector from an input stream. */
    virtual void load(std::vector<double> &vector, std::istream &is);

    virtual void saveState(State const *state, std::ostream &os);
    virtual std::unique_ptr<State> loadState(std::istream &is);

    virtual void saveObservation(Observation const *obs, std::ostream &os);
    virtual std::unique_ptr<Observation> loadObservation(std::istream &is);

    virtual void saveAction(Action const *action, std::ostream &os);
    virtual std::unique_ptr<Action> loadAction(std::istream &is);
};
} /* namespace solver */

#endif /* SOLVER_VECTORTEXTSERIALIZER_HPP_ */
