#ifndef SOLVER_SIMPLEOBSMAP_HPP_
#define SOLVER_SIMPLEOBSMAP_HPP_

#include "ObservationMapping.hpp"

#include <cstddef>

#include <memory>
#include <utility>
#include <unordered_map>

namespace solver {
class BeliefNode;
class SimpleObsMap : public ObservationMapping {
public:
    friend class TextSerializer;
    SimpleObsMap();

    // Default destructor; copying and moving disallowed!
    ~SimpleObsMap();
    SimpleObsMap(SimpleObsMap const &) = delete;
    SimpleObsMap(SimpleObsMap &&) = delete;
    SimpleObsMap &operator=(SimpleObsMap const &) = delete;
    SimpleObsMap &operator=(SimpleObsMap &&) = delete;

    virtual BeliefNode *getBelief(Observation const &obs) const;
    virtual BeliefNode *createBelief(Observation const &obs);
private:
    struct HashContents {
        std::size_t operator()(std::unique_ptr<Observation> const &obs) const {
            return obs->hash();
        }
    };
    struct EqualContents {
        std::size_t operator()(std::unique_ptr<Observation> const &o1,
                std::unique_ptr<Observation> const &o2) const {
            return o1->equals(*o2);
        }
    };
    typedef std::pair<std::unique_ptr<Observation>,
            std::unique_ptr<BeliefNode>> Entry;
    typedef std::unordered_map<std::unique_ptr<Observation>,
            std::unique_ptr<BeliefNode>, HashContents, EqualContents> ObsMap;
    ObsMap obsNodes_;
};
} /* namespace solver */

#endif /* SOLVER_SIMPLEOBSMAP_HPP_ */
