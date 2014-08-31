#ifndef ROS_INTERFACE_HPP_
#define ROS_INTERFACE_HPP_

#include <string>
#include <memory>                   // For unique_ptr
#include <iostream>                     // for cout
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>  // For finding package path

#include "solver/Agent.hpp"
#include "solver/Solver.hpp"
#include "solver/Simulator.hpp"

#include "solver/abstract-problem/Model.hpp"
#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/ModelChange.hpp"
#include "solver/abstract-problem/Observation.hpp"
#include "solver/abstract-problem/Options.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/BeliefTree.hpp"

#include "global.hpp"	// for RandomGenerator

class RosInterface {

public:
	RosInterface();
	~RosInterface();

	/** Load options from file at
	 *  <ROS package path>/problems/<problemName>/<cfgFile>
	 */
	template<typename OptionsType>
	OptionsType loadOptions(std::string problemName,
			std::string cfgFile = "default.cfg",
			std::string packageName = "tapir") {
		std::string problemPath = ros::package::getPath(packageName) +
				"/problems/" + problemName;
		std::string cfgPath = problemPath + "/" + cfgFile;
		std::unique_ptr<options::OptionParser> parser = OptionsType::makeParser(false);
		OptionsType options;
		try {
			parser->setOptions(&options);
			parser->parseCfgFile(cfgPath);
			parser->finalize();

			// Change directory, as code assumes this location for e.g.
			// loading maps from file
			tapir::change_directory(problemPath);
		} catch (options::OptionParsingException const &e) {
			std::cerr << e.what() << std::endl;
		}
		return options;
	}

	/** Initialise models, solver, and simulator (if required). The Simulator
	 *  class is only used if internalSimulation == true, in this case
	 *  resulting observations and states are generated.
	 *  If the models need to be initialised in some way first, they can
	 *  optionally be constructed before this function and provided as arguments,
	 *  otherwise they are constructed internally
	 */
	template<typename ModelType, typename OptionsType>
	void initialise(OptionsType options, bool internalSimulation,
			std::unique_ptr<ModelType> solverModel = nullptr,
			std::unique_ptr<ModelType> simulatorModel = nullptr) {

		// Need to delete pointer if it was initialised before
		if (initialised_ && !internalSimulation_) {
			delete agent_;
		}
		initialised_ = true;
		internalSimulation_ = internalSimulation;
		hasDynamicChanges_ = options.areDynamic;

		// If seed is not specified in options, seed using current time
		if (options.seed == 0) {
			options.seed = std::time(nullptr);
		}
		solverGen_.seed(options.seed);
		solverGen_.discard(10);

		// Init solver
		if (solverModel == nullptr) {
			std::cout << "Global seed: " << options.seed << std::endl;
			solverModel = std::make_unique<ModelType>(
					&solverGen_, std::make_unique<OptionsType>(options));
		}
		solver_ = std::make_unique<solver::Solver>(std::move(solverModel));
		solver_->initializeEmpty();
		double totT;
		double tStart;
		tStart = tapir::clock_ms();
		solver_->improvePolicy();
		totT = tapir::clock_ms() - tStart;
		std::cout << "Total solving time: " << totT << "ms" << std::endl;

		// Init simulator
		if (internalSimulation) {
			simulatorGen_.seed(options.seed);
			simulatorGen_.discard(10000);
			if (simulatorModel == nullptr) {
				simulatorModel = std::make_unique<ModelType>(
						&simulatorGen_, std::make_unique<OptionsType>(options));
			}
			simulator_ = std::make_unique<solver::Simulator>(std::move(simulatorModel),
					solver_.get(), options.areDynamic);
			if (options.hasChanges) {
				simulator_->loadChangeSequence(options.changesPath);
			}
			simulator_->setMaxStepCount(options.nSimulationSteps);
			agent_ = simulator_->getAgent();
		} else {
			agent_ = new solver::Agent(solver_.get());
		}
	}

	/** Step the simulation one step. Returns false if simulation has ended.
	 *  This is only used if initialise was called with internalSimulation = true
	 */
	bool stepSimulation();

	/** Handles changes. Returns false if an error occurs. */
	bool handleChanges(std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
			bool resetTree = false);

	/** Returns the current state. Only valid if internally simulating */
	template<typename StateType>
	StateType getCurrentState() const {
		if (!internalSimulation_) {
			std::cerr << "Error: Cannot get state unless simulating internally."
					<< std::endl;
		}
		solver::State const &state =  *(simulator_->getCurrentState());
		return static_cast<StateType const &>(state);
	}

	/** Returns the "best" action */
	template<typename ActionType>
	std::unique_ptr<ActionType> getPreferredAction() {
		std::unique_ptr<solver::Action> action = agent_->getPreferredAction();
		std::unique_ptr<ActionType> temp(dynamic_cast<ActionType *>(action.release()));
		return std::move(temp);
	}

	/** Get simulator model pointer, casted to ModelType.
	 *  Note that if internally simulating, this model is unused.
	 */
	template<typename ModelType>
	ModelType *getSimulatorModel() const {
		return static_cast<ModelType *>(simulator_->getModel());
	}

	/** Get solver model pointer, casted to ModelType */
	template<typename ModelType>
	ModelType *getSolverModel() const {
		return static_cast<ModelType *>(solver_->getModel());
	}

	void improvePolicy() {
		solver::BeliefNode *currentBelief = agent_->getCurrentBelief();
		solver_->improvePolicy(currentBelief);
	}

	/** Update the current belief with the last action taken and the
	 *  resulting observation. If replenish = true, particles are replenished
	 *  before the update
	 */
	void updateBelief(const solver::Action& action,
			const solver::Observation& observation, bool replenish = true) {
		solver::BeliefNode *currentBelief = agent_->getCurrentBelief();
		solver_->replenishChild(currentBelief, action, observation);
		agent_->updateBelief(action, observation);
	}

	/** Getters */
	solver::Model *getSolverModel() const;
	solver::Model *getSimulatorModel() const;
	solver::Agent *getAgent() const;
	solver::Solver *getSolver() const;
	solver::Simulator *getSimulator() const;

private:
	std::unique_ptr<solver::Solver> solver_;
	std::unique_ptr<solver::Simulator> simulator_;
	solver::Agent* agent_;

	RandomGenerator solverGen_;
	RandomGenerator simulatorGen_;

	bool initialised_;
	bool internalSimulation_;
	bool hasDynamicChanges_;

};

#endif /* ROS_INTERFACE_HPP_ */
