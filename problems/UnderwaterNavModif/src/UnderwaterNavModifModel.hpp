#ifndef UNDERWATERNAVMODIFMODEL_HPP
#define UNDERWATERNAVMODIFMODEL_HPP

#include <map>                          // for map
#include <ostream>                      // for operator<<, ostream, basic_ostream, basic_ostream<>::__ostream_type
#include <string>                       // for string
#include <vector>                       // for vector, vector<>::iterator
#include <boost/program_options.hpp>    // for program_options, variables_map
#include "ChangeType.hpp"               // for ChangeType
#include "Model.hpp"                    // for Model
#include "Observation.hpp"              // for Observation
#include "State.hpp"                    // for State
class StRoadmap;

namespace po = boost::program_options;

class UnderwaterNavModifModel: public Model {
public:
    enum Action
        : int {
            EAST = 0, NORTH = 1, SOUTH = 2, NORTHEAST = 3, SOUTHEAST = 4
    };

    void dispAct(unsigned long actId, std::ostream &os) {
        switch (actId) {
        case EAST:
            os << "EAST";
            break;
        case NORTH:
            os << "NORTH";
            break;
        case SOUTH:
            os << "SOUTH";
            break;
        case NORTHEAST:
            os << "NORTHEAST";
            break;
        case SOUTHEAST:
            os << "SOUTHEAST";
            break;
        default:
            os << "ERROR-" << actId;
            break;
        }
    }

    void dispState(State &s, std::ostream &os) {
        os << "(" << s[0] << ", " << s[1] << ")";
    }

    void dispObs(Observation &o, std::ostream &os) {
        if (o[0] == -1 && o[1] == -1) {
            os << "NONE";
            return;
        }
        os << "(" << o[0] << ", " << o[1] << ")";
    }

    UnderwaterNavModifModel(po::variables_map vm);
    ~UnderwaterNavModifModel();
    UnderwaterNavModifModel(const UnderwaterNavModifModel&) = delete;
    UnderwaterNavModifModel(UnderwaterNavModifModel&) = delete;
    UnderwaterNavModifModel &operator=(const UnderwaterNavModifModel&) = delete;
    UnderwaterNavModifModel &operator=(UnderwaterNavModifModel&) = delete;

    /***** Start implementation of Model's virtual methods *****/
    // Simple getters
    double getDiscount() {
        return discount;
    }
    unsigned long getNActions() {
        return nActions;
    }
    unsigned long getNObservations() {
        return nObservations;
    }
    unsigned long getNStVars() {
        return nStVars;
    }
    double getMinVal() {
        return minVal;
    }
    double getMaxVal() {
        return maxVal;
    }

    unsigned long getNParticles() {
        return nParticles;
    }
    long getMaxTrials() {
        return maxTrials;
    }
    double getDepthTh() {
        return depthTh;
    }
    double getExploreCoef() {
        return exploreCoef;
    }
    long getMaxDistTry() {
        return maxDistTry;
    }
    double getDistTh() {
        return distTh;
    }

    // Other virtual methods
    void sampleAnInitState(State &tmpStVals);
    bool isTerm(State &sVals);
    void solveHeuristic(State &s, double *qVal);
    double getDefaultVal();

    bool getNextState(State &currStVals, unsigned long actId,
            double *immediateRew, State &nxtSVals, Observation &obs);
    double getReward(State &sVals);
    double getReward(State &sVals, unsigned long actId);

    void getStatesSeeObs(unsigned long actId, Observation &obs,
            std::vector<State> &partSt, std::vector<State> &partNxtSt);
    void getStatesSeeObs(unsigned long actId, Observation &obs,
            std::vector<State> &partNxtSt);

    void setChanges(const char* chName, std::vector<long> &chTime);
    void update(long tCh, std::vector<State> &affectedRange,
            std::vector<ChangeType> &typeOfChanges);
    bool modifStSeq(std::vector<State> &seqStVals, long startAffectedIdx,
            long endAffectedIdx, std::vector<State> &modifStSeq,
            std::vector<long> &modifActSeq,
            std::vector<Observation> &modifObsSeq,
            std::vector<double> &modifRewSeq);

    void drawEnv(std::ostream &os);
    void drawState(State &s, std::ostream &os);

    // Additional initialisation.
    void setInitObsGoal();

private:
    // Problem parameters.
    double discount;
    unsigned long nActions, nObservations, nStVars;
    double minVal, maxVal;

    // SBT parameters
    unsigned long nParticles;
    long maxTrials;
    double depthTh;
    double exploreCoef;

    long maxDistTry;
    double distTh;

    /** The number of state particles in the initial belief. */
    unsigned long nInitBel;
    /** A vector of all the states in the initial belief. */
    std::vector<State> initBel;

    unsigned long nX, nY, nGoals, nRocks;
    double goalReward, crashPenalty, moveCost, moveDiagCost;
    double ctrlCorrectProb, ctrlErrProb1;
    double rolloutExploreTh;
    std::vector<std::string> envMap;
    std::vector<State> goals;
    std::vector<State> rocks;
    std::vector<State> allObservations;
    std::map<long, std::map<long, short> > cellType;
    // 0: usual
    // 1: goals
    // 2: rocks
    // 3: observation
    // 4: spc. reward
    // 5: obstacle.
    short nSpcRew;
    std::vector<double> spcRew;
    std::map<long, std::vector<std::string> > changes;
    std::vector<State> obstacleRegion;

    StRoadmap *roadmap;
    long nTryCon, maxDistCon, nVerts;

    //double getExpDist(State &s, long firstAct);
    double getDist(State &s1, State &s2);
    void getNextState(State &s, unsigned long actId, State &sp);
    void inObsRegion(State &st, Observation &obs);
    double getDistToNearestGoal(State &st);
    double getDistToNearestObs(State &st, State &nxtSt);
    bool inGoal(State &st);
    bool inRock(State &st);
    void getReachableSt(State &s, unsigned long actId,
            std::vector<State> &nxtS);
    std::vector<State>::iterator getIterator(std::vector<State> &vecStVals,
            long x, long y);

    int findCollision(State &s);

};

#endif
