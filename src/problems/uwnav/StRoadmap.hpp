#ifndef STROADMAP_HPP_
#define STROADMAP_HPP_

#include <map>                          // for map
#include <ostream>                      // for ostream
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "defs.hpp"                     // for RandomGenerator
#include "solver/State.hpp"                    // for State
#include "UnderwaterNavState.hpp"
#include "uwnav.hpp"

namespace uwnav {
class StRoadmap {
  public:
    StRoadmap(RandomGenerator *randGen, std::vector<UnderwaterNavState> &goals, long nVerts, long nGoalsSamp,
              long nTryCon, long maxDistCon,
              std::map<long, std::map<long, UnderwaterNavCellType> > &env, long nX, long nY);
    ~StRoadmap() = default;
    StRoadmap(StRoadmap const &) = delete;
    StRoadmap(StRoadmap &) = delete;
    StRoadmap &operator=(StRoadmap const &) = delete;
    StRoadmap &operator=(StRoadmap &) = delete;

    void updateRoadmap(std::map<long, std::map<long, short> > &env_,
                       std::vector<UnderwaterNavState> &goals, long nGoalsSamp);
    double getDistToGoal(UnderwaterNavState &startSt);
    void draw(std::ostream &os);

  private:
    RandomGenerator *randGen_;
    long nCols_, nRows_;
    std::map<long, std::map<long, UnderwaterNavCellType> > env_;
    std::map<long, long> weight_;
    long totW_, maxTryCon_, maxDistCon_;

    long nVerts_, maxVerts_, lastGoalIdx_;
    std::vector<UnderwaterNavState> milestones_;

    // e: (fromVertIdx, toVertIdx, cost)
    std::map<long, std::vector<std::pair<long, long> > > outEdges_, inEdges_;
    std::map<long, long> shortestDistToGoal_;

    void setWeight();

    void insertGoalMilestones(std::vector<UnderwaterNavState> &goals, long nGoalsSamp);
    UnderwaterNavState sampleAMilestone();
    bool insertMilestone(UnderwaterNavState milestone);
    bool hasMilestone(UnderwaterNavState milestone);
    void insertMyMilestones();

    long lineSegOk(UnderwaterNavState &st1, UnderwaterNavState &st2);
    double dist(UnderwaterNavState &s1, UnderwaterNavState &s2);
    void getDistToGoal();
};
} /* namespace uwnav */
#endif /* STROADMAP_HPP_ */
