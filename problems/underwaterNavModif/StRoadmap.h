#ifndef StRoadmap_H
#define StRoadmap_H

#include <utility> 
#include <vector>
#include <map>
#include "Model.h"
#include "GlobalResources.h"

using namespace std;

class StRoadmap {
	public:
		StRoadmap(vector<StateVals> &goals, long nVerts_, long nGoalsSamp, long nTryCon_, long maxDistCon_,
				map< long, map<long, short> > &env_, long nX_, long nY_);
		~StRoadmap();
		
		void updateRoadmap(map< long, map<long, short> > &env_, vector<StateVals> &goals, long nGoalsSamp);
		double getDistToGoal(StateVals &startSt);
		void draw(ostream &os);

	private:
		long nX, nY;
		map< long, map<long, short> > env;
		map<long, long> weight;
		long totW, maxTryCon, maxDistCon;
		
		long nVerts, maxVerts, lastGoalIdx;
		vector<StateVals> V;
		map< long, vector< pair<long, long> > > outEdges, inEdges;	// e: (fromVertIdx, toVertIdx, cost)
		map<long, long> shortestDistToGoal;
		
		void setWeight();
		void insertGoalMilestones(vector<StateVals> &goals, long nGoalsSamp);
		void sampleAMilestone(StateVals &st);
		bool insertMilestone(StateVals &st);
		long lineSegOk(StateVals &st1, StateVals &st2);
		double dist(StateVals &s1, StateVals &s2);
		void getDistToGoal();
	
		bool VContains(long x, long y);
		
		void insertMyMilestones();
};
#endif
