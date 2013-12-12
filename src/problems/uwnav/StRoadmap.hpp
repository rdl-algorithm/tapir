#ifndef STROADMAP_HPP_
#define STROADMAP_HPP_

#include <map>                          // for map
#include <ostream>                      // for ostream
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "defs.hpp"                     // for RandomGenerator
#include "State.hpp"                    // for State

class StRoadmap {
  public:
    StRoadmap(RandomGenerator *randGen, std::vector<VectorState> &goals, long nVerts, long nGoalsSamp,
              long nTryCon, long maxDistCon,
              std::map<long, std::map<long, short> > &env, long nX, long nY);
    ~StRoadmap();
    StRoadmap(StRoadmap const &) = delete;
    StRoadmap(StRoadmap &) = delete;
    StRoadmap &operator=(StRoadmap const &) = delete;
    StRoadmap &operator=(StRoadmap &) = delete;

    void updateRoadmap(std::map<long, std::map<long, short> > &env_,
                       std::vector<VectorState> &goals, long nGoalsSamp);
    double getDistToGoal(VectorState &startSt);
    void draw(std::ostream &os);

  private:
    RandomGenerator *randGen;
    long nX, nY;
    std::map<long, std::map<long, short> > env;
    std::map<long, long> weight;
    long totW, maxTryCon, maxDistCon;

    long nVerts, maxVerts, lastGoalIdx;
    std::vector<VectorState> V;

    // e: (fromVertIdx, toVertIdx, cost)
    std::map<long, std::vector<std::pair<long, long> > > outEdges, inEdges;
    std::map<long, long> shortestDistToGoal;

    void setWeight();
    void insertGoalMilestones(std::vector<VectorState> &goals, long nGoalsSamp);
    void sampleAMilestone(VectorState &st);
    bool insertMilestone(VectorState &st);
    long lineSegOk(VectorState &st1, VectorState &st2);
    double dist(VectorState &s1, VectorState &s2);
    void getDistToGoal();

    bool VContains(long x, long y);

    void insertMyMilestones();
};
#endif /* STROADMAP_HPP_ */
