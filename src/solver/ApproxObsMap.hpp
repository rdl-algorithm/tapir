#ifndef SOLVER_APPROXOBSMAP_HPP_
#define SOLVER_APPROXOBSMAP_HPP_

#include "ObservationMapping.hpp"

#include <cstddef>

#include <memory>
#include <utility>
#include <unordered_map>
#include <vector>

namespace solver {
class BeliefNode;
class Model;

class ApproxObsMap : public ObservationMapping {
public:
    friend class TextSerializer;
    ApproxObsMap(Model *model, double maxDistance);

    // Default destructor; copying and moving disallowed!
    ~ApproxObsMap();
    ApproxObsMap(ApproxObsMap const &) = delete;
    ApproxObsMap(ApproxObsMap &&) = delete;
    ApproxObsMap &operator=(ApproxObsMap const &) = delete;
    ApproxObsMap &operator=(ApproxObsMap &&) = delete;

    virtual BeliefNode *getBelief(Observation const &obs) const;
    virtual BeliefNode *createBelief(Observation const &obs);
private:
    Model *model_;
    double maxDistance_;
    typedef std::pair<std::unique_ptr<Observation>,
            std::unique_ptr<BeliefNode>> Entry;
    typedef std::vector<Entry> ObsVector;
    ObsVector obsNodes_;
};
} /* namespace solver */

#endif /* SOLVER_APPROXOBSMAP_HPP_ */
