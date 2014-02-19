#ifndef SOLVER_DISCRETEOBSERVATIONMAP_HPP_
#define SOLVER_DISCRETEOBSERVATIONMAP_HPP_

#include "ObservationMapping.hpp"

#include <cstddef>

#include <memory>
#include <utility>
#include <unordered_map>
#include <vector>

namespace solver {
class BeliefNode;
class Model;

class DiscreteObservationMap : public ObservationMapping {
public:
    friend class TextSerializer;
    DiscreteObservationMap(Model *model);

    // Default destructor; copying and moving disallowed!
    ~DiscreteObservationMap();
    DiscreteObservationMap(DiscreteObservationMap const &) = delete;
    DiscreteObservationMap(DiscreteObservationMap &&) = delete;
    DiscreteObservationMap &operator=(DiscreteObservationMap const &) = delete;
    DiscreteObservationMap &operator=(DiscreteObservationMap &&) = delete;

    virtual BeliefNode *getBelief(Observation const &obs) const;
    virtual BeliefNode *createBelief(Observation const &obs);
private:
    Model *model_;

    struct HashContents {
        std::size_t operator()(Observation const *obs) const {
            return obs->hash();
        }
    };
    struct EqualContents {
        std::size_t operator()(Observation const *o1,
                Observation const *o2) const {
            return o1->equals(*o2);
        }
    };

    std::vector<std::unique_ptr<Observation>> observations_;
    typedef std::unordered_map<Observation const *,
            std::unique_ptr<BeliefNode>, HashContents, EqualContents> ObsMap;
    ObsMap obsNodes_;
};
} /* namespace solver */

#endif /* SOLVER_DISCRETEOBSERVATIONMAP_HPP_ */
