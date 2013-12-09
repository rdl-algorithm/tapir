#ifndef UNDERWATERNAVMODIFMODEL_HPP
#define UNDERWATERNAVMODIFMODEL_HPP

#include <map>                          // for map
#include <ostream>                      // for operator<<, ostream, basic_ostream, basic_ostream<>::__ostream_type
#include <string>                       // for string
#include <vector>                       // for vector, vector<>::iterator

#include <boost/program_options.hpp>    // for program_options, variables_map

#include "defs.hpp"                     // for RandomGenerator
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

    void dispState(VectorState &s, std::ostream &os) {
        os << "(" << s[0] << ", " << s[1] << ")";
    }

    void dispObs(Observation &o, std::ostream &os) {
        if (o[0] == -1 && o[1] == -1) {
            os << "NONE";
            return;
        }
        os << "(" << o[0] << ", " << o[1] << ")";
    }

    UnderwaterNavModifModel(RandomGenerator *randGen, po::variables_map vm);
    ~UnderwaterNavModifModel();
    UnderwaterNavModifModel(UnderwaterNavModifModel const &) = delete;
    UnderwaterNavModifModel(UnderwaterNavModifModel &&) = delete;
    UnderwaterNavModifModel &operator=(UnderwaterNavModifModel const &) = delete;
    UnderwaterNavModifModel &operator=(UnderwaterNavModifModel &&) = delete;

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
    void sampleAnInitState(VectorState &tmpStVals);
    bool isTerm(VectorState &sVals);
    void solveHeuristic(VectorState &s, double *qVal);
    double getDefaultVal();

    bool getNextState(VectorState &currStVals, unsigned long actId,
                      double *immediateRew, VectorState &nxtSVals, Observation &obs);
    double getReward(VectorState &sVals);
    double getReward(VectorState &sVals, unsigned long actId);

    void getStatesSeeObs(unsigned long actId, Observation &obs,
                         std::vector<VectorState> &partSt, std::vector<VectorState> &partNxtSt);
    void getStatesSeeObs(unsigned long actId, Observation &obs,
                         std::vector<VectorState> &partNxtSt);

    void getChangeTimes(char const *chName, std::vector<long> &chTime);
    void update(long tCh, std::vector<VectorState> &affectedRange,
                std::vector<ChangeType> &typeOfChanges);
    bool modifStSeq(std::vector<VectorState> &seqStVals, long startAffectedIdx,
                    long endAffectedIdx, std::vector<VectorState> &modifStSeq,
                    std::vector<long> &modifActSeq,
                    std::vector<Observation> &modifObsSeq,
                    std::vector<double> &modifRewSeq);

    void drawEnv(std::ostream &os);
    void drawState(VectorState &s, std::ostream &os);

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
    std::vector<VectorState> initBel;

    unsigned long nX, nY, nGoals, nRocks;
    double goalReward, crashPenalty, moveCost, moveDiagCost;
    double ctrlCorrectProb, ctrlErrProb1;
    double rolloutExploreTh;
    std::vector<std::string> envMap;
    std::vector<VectorState> goals;
    std::vector<VectorState> rocks;
    std::vector<VectorState> allObservations;
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
    std::vector<VectorState> obstacleRegion;

    StRoadmap *roadmap;
    long nTryCon, maxDistCon, nVerts;

    //double getExpDist(State &s, long firstAct);
    double getDist(VectorState &s1, VectorState &s2);
    void getNextState(VectorState &s, unsigned long actId, VectorState &sp);
    void inObsRegion(VectorState &st, Observation &obs);
    double getDistToNearestGoal(VectorState &st);
    double getDistToNearestObs(VectorState &st, VectorState &nxtSt);
    bool inGoal(VectorState &st);
    bool inRock(VectorState &st);
    void getReachableSt(VectorState &s, unsigned long actId,
                        std::vector<VectorState> &nxtS);
    std::vector<VectorState>::iterator getIterator(std::vector<VectorState> &vecStVals,
            long x, long y);

    int findCollision(VectorState &s);

};

#endif
