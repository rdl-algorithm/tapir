#ifndef SOLVER_VECTORLPTEXTSERIALIZER_HPP_
#define SOLVER_VECTORLPTEXTSERIALIZER_HPP_

#include "Serializer.hpp"

#include "global.hpp"

namespace solver {
/** Simple vector-based serialization of States, Observations, and Actions;
 * using an L1 metric where needed.
 * If you use a custom subclass of State/Observation/Action you will need
 * to override the serialization for that subclass.
 */
class VectorLPTextSerializer : virtual public Serializer {
  public:
    VectorLPTextSerializer() = default;
    virtual ~VectorLPTextSerializer() = default;
    _NO_COPY_OR_MOVE(VectorLPTextSerializer);

    /** Saves a vector to an output stream. */
    virtual void save(std::vector<double> const &vector, std::ostream &os);
    /** Loads a vector from an input stream. */
    virtual void load(std::vector<double> &vector, std::istream &is);

    virtual void saveState(State const *state, std::ostream &os) override;
    virtual std::unique_ptr<State> loadState(std::istream &is) override;

    virtual void saveObservation(Observation const *obs,
            std::ostream &os) override;
    virtual std::unique_ptr<Observation> loadObservation(
            std::istream &is) override;

    virtual void saveAction(Action const *action, std::ostream &os) override;
    virtual std::unique_ptr<Action> loadAction(std::istream &is) override;
};
} /* namespace solver */

#endif /* SOLVER_VECTORLPTEXTSERIALIZER_HPP_ */
