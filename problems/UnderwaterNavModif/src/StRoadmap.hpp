#ifndef STROADMAP_HPP
#define STROADMAP_HPP

#include <map>
#include <iosfwd>
#include <utility>
#include <vector>

#include "State.hpp"

class StRoadmap {
public:
    StRoadmap(std::vector<State> &goals, long nVerts, long nGoalsSamp,
            long nTryCon, long maxDistCon,
            std::map<long, std::map<long, short> > &env, long nX, long nY);
    ~StRoadmap();
    StRoadmap(const StRoadmap&) = delete;
    StRoadmap(StRoadmap&) = delete;
    StRoadmap &operator=(const StRoadmap&) = delete;
    StRoadmap &operator=(StRoadmap&) = delete;

    void updateRoadmap(std::map<long, std::map<long, short> > &env_,
            std::vector<State> &goals, long nGoalsSamp);
    double getDistToGoal(State &startSt);
    void draw(std::ostream &os);

private:
    long nX, nY;
    std::map<long, std::map<long, short> > env;
    std::map<long, long> weight;
    long totW, maxTryCon, maxDistCon;

    long nVerts, maxVerts, lastGoalIdx;
    std::vector<State> V;
    std::map<long, std::vector<std::pair<long, long> > > outEdges, inEdges; // e: (fromVertIdx, toVertIdx, cost)
    std::map<long, long> shortestDistToGoal;

    void setWeight();
    void insertGoalMilestones(std::vector<State> &goals, long nGoalsSamp);
    void sampleAMilestone(State &st);
    bool insertMilestone(State &st);
    long lineSegOk(State &st1, State &st2);
    double dist(State &s1, State &s2);
    void getDistToGoal();

    bool VContains(long x, long y);

    void insertMyMilestones();
};
#endif
