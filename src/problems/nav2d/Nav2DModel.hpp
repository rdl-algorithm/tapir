#ifndef NAV2DMODEL_HPP_
#define NAV2DMODEL_HPP_

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <string>                       // for string
#include <utility>                      // for pair
#include <vector>                       // for vector

#include <boost/program_options.hpp>    // for variables_map

#include "global.hpp"                     // for RandomGenerator
#include "problems/shared/GridPosition.hpp"  // for GridPosition
#include "problems/shared/ModelWithProgramOptions.hpp"  // for ModelWithProgramOptions
#include "solver/Action.hpp"            // for Action
#include "solver/ChangeFlags.hpp"        // for ChangeFlags
#include "solver/Model.hpp"             // for Model::StepResult, Model
#include "solver/Observation.hpp"       // for Observation
#include "solver/State.hpp"

namespace po = boost::program_options;

namespace solver {
class StatePool;
} /* namespace solver */

namespace nav2d {
class Nav2DState;

class Nav2DModel : public ModelWithProgramOptions {
  public:
    Nav2DModel(RandomGenerator *randGen, po::variables_map vm);
    ~Nav2DModel() = default;
    Nav2DModel(Nav2DModel const &) = delete;
    Nav2DModel(Nav2DModel &&) = delete;
    Nav2DModel &operator=(Nav2DModel const &) = delete;
    Nav2DModel &operator=(Nav2DModel &&) = delete;

    /** Enumerates the possible actions */
    enum Nav2DAction : long {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3,
        NAV2D = 4
    };

    std::string getName() {
        return "Nav2D";
    }

    /***** Start implementation of Model's virtual methods *****/
    // Simple getters
    long getNActions() {
        return nActions_;
    }
    long getNStVars() {
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
    /** Generates a new nav2d state uniformly at random. */
    std::unique_ptr<solver::State> sampleStateUniform();

    bool isTerminal(solver::State const &state);
    double getHeuristicValue(solver::State const &state);
    double getDefaultVal();

    /* --------------- Black box dynamics ----------------- */
    virtual std::unique_ptr<solver::State> generateNextState(
            solver::State const &state, solver::Action const &action);
    virtual std::unique_ptr<solver::Observation> generateObservation(
            solver::Action const &action, solver::State const &nextState);
    virtual double getReward(solver::State const &state,
                solver::Action const &action);
    virtual Model::StepResult generateStep(solver::State const &state,
            solver::Action const &action);

    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action,
            solver::Observation const &obs,
            std::vector<solver::State const *> const &previousParticles);
    std::vector<std::unique_ptr<solver::State>> generateParticles(
            solver::Action const &action,
            solver::Observation const &obs);

    std::vector<long> loadChanges(char const *changeFilename);
    void update(long time, solver::StatePool *pool);

    void dispAct(solver::Action const &action, std::ostream &os);
    void dispObs(solver::Observation const &obs, std::ostream &os);
    void drawEnv(std::ostream &os);
    void drawState(solver::State const &state, std::ostream &os);

  private:
    /** Initialises the required data structures and variables */
    void initialise();

    // General problem parameters
    long nActions_, nStVars_;
    double minVal_, maxVal_;
};
} /* namespace nav2d */

#endif /* NAV2DMODEL_HPP_ */
