#ifndef StRoadmap_H
#define StRoadmap_H

#include <ostream>
#include <utility>
#include <vector>
#include <map>

#include "Model.h"
#include "GlobalResources.h"

class StRoadmap {
public:
    StRoadmap(std::vector<StateVals> &goals, long nVerts, long nGoalsSamp,
            long nTryCon, long maxDistCon,
            std::map<long, std::map<long, short> > &env, long nX, long nY);
    ~StRoadmap();

    void updateRoadmap(std::map<long, std::map<long, short> > &env_,
            std::vector<StateVals> &goals, long nGoalsSamp);
    double getDistToGoal(StateVals &startSt);
    void draw(std::ostream &os);

private:
    long nX, nY;
    std::map<long, std::map<long, short> > env;
    std::map<long, long> weight;
    long totW, maxTryCon, maxDistCon;

    long nVerts, maxVerts, lastGoalIdx;
    std::vector<StateVals> V;
    std::map<long, std::vector<std::pair<long, long> > > outEdges, inEdges; // e: (fromVertIdx, toVertIdx, cost)
    std::map<long, long> shortestDistToGoal;

    void setWeight();
    void insertGoalMilestones(std::vector<StateVals> &goals, long nGoalsSamp);
    void sampleAMilestone(StateVals &st);
    bool insertMilestone(StateVals &st);
    long lineSegOk(StateVals &st1, StateVals &st2);
    double dist(StateVals &s1, StateVals &s2);
    void getDistToGoal();

    bool VContains(long x, long y);

    void insertMyMilestones();
};
#endif
