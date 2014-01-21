#ifndef UNDERWATERNAVMODIFMODEL_HPP_
#define UNDERWATERNAVMODIFMODEL_HPP_

#include <map>                          // for map
#include <ostream>                      // for operator<<, ostream, basic_ostream, basic_ostream<>::__ostream_type
#include <string>                       // for string
#include <vector>                       // for vector, vector<>::iterator

#include <boost/program_options.hpp>    // for program_options, variables_map

#include "defs.hpp"                     // for RandomGenerator
#include "problems/shared/GridPosition.hpp"
#include "problems/shared/ModelWithProgramOptions.hpp"
#include "solver/Action.hpp"            // for Action
#include "solver/ChangeType.hpp"               // for ChangeType
#include "solver/Model.hpp"                    // for Model
#include "solver/Observation.hpp"              // for Observation
#include "uwnav.hpp"

class StRoadmap;
class UnderwaterNavState;

namespace po = boost::program_options;

namespace uwnav {
class UnderwaterNavModel: public ModelWithProgramOptions {
  public:
    UnderwaterNavModel(RandomGenerator *randGen, po::variables_map vm);
    ~UnderwaterNavModel() = default;
    UnderwaterNavModel(UnderwaterNavModel const &) = delete;
    UnderwaterNavModel(UnderwaterNavModel &&) = delete;
    UnderwaterNavModel &operator=(UnderwaterNavModel const &) = delete;
    UnderwaterNavModel &operator=(UnderwaterNavModel &&) = delete;

    /***** Start implementation of Model's virtual methods *****/
       // Simple getters
       unsigned long getNActions() {
           return nActions_;
       }
       unsigned long getNObservations() {
           return nObservations_;
       }
       unsigned long getNStVars() {
           return nStVars_;
       }
       double getMinVal() {
           return minVal_;
       }
       double getMaxVal() {
           return maxVal_;
       }

       // Other virtual methods
    std::unique_ptr<solver::State> sampleAnInitState();

    bool isTerm(solver::State const &state);
    double solveHeuristic(solver::State const &state);
    double getDefaultVal();

    solver::Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action);
    double getReward(solver::State const &state);
    double getReward(solver::State const &state, solver::Action const &action);

    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action, solver::Observation const &obs,
            std::vector<solver::State *> const &previousParticles);
    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action, solver::Observation const &obs);

    std::vector<long> loadChanges(char const *changeFilename);
    void update(long time,
            std::vector<std::unique_ptr<solver::State>> *affectedRange,
            std::vector<solver::ChangeType> *typeOfChanges);

    bool modifStSeq(std::vector<solver::State const *> const &states,
            long startAffectedIdx, long endAffectedIdx,
            std::vector<std::unique_ptr<solver::State>> *modifStSeq,
            std::vector<solver::Action> *modifActSeq,
            std::vector<solver::Observation> *modifObsSeq,
            std::vector<double> *modifRewSeq);

    void dispAct(solver::Action const &action, std::ostream &os);
    /** Displays a single cell of the map. */
    void dispCell(UnderwaterNavCellType cellType, std::ostream &os);
    void dispObs(solver::Observation const &obs, std::ostream &os);
    void drawEnv(std::ostream &os);
    void drawState(solver::State const &state, std::ostream &os);

  private:
    // uwnav-specific parameters
    unsigned long nX_, nY_;
    double goalReward_, crashPenalty_, moveCost_, moveDiagCost_;
    double ctrlCorrectProb_, ctrlErrProb1_;
    double rolloutExploreTh_;
    long nTryCon_, maxDistCon_, nVerts_;

    // General parameters.
    unsigned long nActions_, nObservations_, nStVars_;
    double minVal_, maxVal_;

    unsigned long nGoals_;
    unsigned long nRocks_;
    /** The number of state particles in the initial belief. */
    unsigned long nInitBel_;
    /** A vector of all the states in the initial belief. */
    std::vector<UnderwaterNavState> initBel_;

    short nSpcRew_;
    std::vector<double> spcRew_;

    std::vector<std::string> mapText_;
    std::map<long, std::map<long, UnderwaterNavCellType> > envMap_;
    std::unique_ptr<StRoadmap> roadmap_;

    std::map<long, std::vector<std::string> > changes;

    std::vector<UnderwaterNavState> goals_;
    std::vector<UnderwaterNavState> rocks_;
    std::vector<UnderwaterNavState> allObservations_;
    std::vector<UnderwaterNavState> obstacleRegion_;

    void setInitObsGoal();
    double getDist(UnderwaterNavState const &s1, UnderwaterNavState const &s2);
    UnderwaterNavState getNextState(UnderwaterNavState const &s,
            solver::Action const &action);
    void inObsRegion(UnderwaterNavState &st, solver::Observation &obs);
    double getDistToNearestGoal(UnderwaterNavState &st);
    double getDistToNearestObs(UnderwaterNavState &st, UnderwaterNavState &nxtSt);
    bool inGoal(UnderwaterNavState &st);
    bool inRock(UnderwaterNavState &st);
    void getReachableSt(UnderwaterNavState &s, unsigned long actId,
                        std::vector<UnderwaterNavState> &nxtS);
    std::vector<UnderwaterNavState>::iterator getIterator(std::vector<UnderwaterNavState> &vecStVals,
            long x, long y);

    int findCollision(UnderwaterNavState &s);
};
} /* namespace uwnav */

#endif /* UNDERWATERNAVMODIFMODEL_HPP_ */
