/** @file TapirNode.hpp
 *
 * Defines TapirNode, an abstract class that can be used to implement the
 * TAPIR solver as a ROS node.
 */

#ifndef TAPIR_NODE_HPP_
#define TAPIR_NODE_HPP_

#include <memory>                   // For unique_ptr
#include <iostream>                     // for cout

#include <ros/ros.h>
#include <ros/package.h>  // For finding package path

#include "global.hpp"
#include "solver/Agent.hpp"
#include "solver/Solver.hpp"
#include "solver/Simulator.hpp"
#include "solver/BeliefTree.hpp"

template <class Options, class Model, class Observation, class Action>
class TapirNode {

protected:
	std::unique_ptr<ros::NodeHandle> node_;

	solver::Agent* agent_;
	Model* solverModel_;
	std::unique_ptr<solver::Solver> solver_;
	std::unique_ptr<solver::Simulator> simulator_;

	Options options_;

	RandomGenerator solverGen_;
	RandomGenerator simulatorGen_;

	bool internalSimulation_;
	std::string rosNamespace_;
	long stepNumber_;
	bool finished_;
	std::unique_ptr<Observation> lastObservation_;
	std::unique_ptr<Action> lastAction_;

	/** Return a current observation. Left for user implementation  */
	virtual std::unique_ptr<Observation> getObservation() {
		return nullptr;
	}

	/** Apply an action. Left for user implementation */
	virtual void applyAction(const Action& action) {}

	/** Load options from file. problemPath should be path to folder containing
	 *  configuration files, default <ROS package path>/problems/<problemName>
	 *  cfgFile is the cfg file to load, the default is default.cfg
	 */
	virtual void loadOptions(std::string problemPath,
			std::string cfgFile = "default.cfg") {
		std::string cfgPath = problemPath + "/" + cfgFile;
		std::unique_ptr<options::OptionParser> parser = Options::makeParser(false);
		try {
			parser->setOptions(&options_);
			parser->parseCfgFile(cfgPath);
			parser->finalize();

			// Change directory, as code assumes this location for e.g.
			// loading maps from file
			tapir::change_directory(problemPath);
		} catch (options::OptionParsingException const &e) {
			std::cerr << e.what() << std::endl;
		}
	}

	/** Initialise models, solver, and simulator (if required). The Simulator
	 *  class is only used if internalSimulation == true, in this case
	 *  resulting observations and states are generated.
	 */
	virtual void initTapir() {

		// If seed is not specified in options, seed using current time
		if (options_.seed == 0) {
			options_.seed = std::time(nullptr);
		}
		solverGen_.seed(options_.seed);
		solverGen_.discard(10);

		// Init solver
		std::cout << "Global seed: " << options_.seed << std::endl;
		std::unique_ptr<Model> solverModel = std::make_unique<Model>(
				&solverGen_, std::make_unique<Options>(options_));
		solver_ = std::make_unique<solver::Solver>(std::move(solverModel));
		solver_->initializeEmpty();
		solverModel_ = static_cast<Model *>(solver_->getModel());

		// Generate policy
		double totT;
		double tStart;
		tStart = tapir::clock_ms();
		solver_->improvePolicy();
		totT = tapir::clock_ms() - tStart;
		std::cout << "Total solving time: " << totT << "ms" << std::endl;

		// Init simulator
		if (internalSimulation_) {
			simulatorGen_.seed(options_.seed);
			simulatorGen_.discard(10000);
			std::unique_ptr<Model> simModel = std::make_unique<Model>(
					&simulatorGen_, std::make_unique<Options>(options_));
			simulator_ = std::make_unique<solver::Simulator>(
					std::move(simModel), solver_.get(), options_.areDynamic);
			if (options_.hasChanges) {
				simulator_->loadChangeSequence(options_.changesPath);
			}
			simulator_->setMaxStepCount(options_.nSimulationSteps);
			agent_ = simulator_->getAgent();
		} else {
			agent_ = new solver::Agent(solver_.get());
		}
	}

	/** Tapir processing, typically called in a timer loop */
	virtual void processTapir() {
		if (internalSimulation_) {
			finished_ = !simulator_->stepSimulation();
			return;
		}
		if (stepNumber_ != 0) {
			lastObservation_ = getObservation();
			solver::BeliefNode *currentBelief = agent_->getCurrentBelief();
			solver_->replenishChild(currentBelief, *lastAction_, *lastObservation_);
			agent_->updateBelief(*lastAction_, *lastObservation_);
			currentBelief = agent_->getCurrentBelief();

			// If we're pruning on every step, we do it now.
			if (options_.pruneEveryStep) {
				double pruningTimeStart = tapir::clock_ms();
				long nSequencesDeleted = solver_->pruneSiblings(currentBelief);
				long pruningTime = tapir::clock_ms() - pruningTimeStart;
				if (options_.hasVerboseOutput) {
					std::cout << "Pruned " << nSequencesDeleted << " sequences in ";
					std::cout << pruningTime << "ms." << std::endl;
				}
			}
		}
		stepNumber_++;

		// Get preferred action. Needs to be cast into problem's action
		std::unique_ptr<solver::Action> sa = agent_->getPreferredAction();
		if (sa == nullptr) {
			std::cout << "ERROR: Could not choose an action." << std::endl;
		} else {
			lastAction_.reset(static_cast<Action *>(sa.release()));
			applyAction(*lastAction_);
		}
		solver::BeliefNode *currentBelief = agent_->getCurrentBelief();
		solver_->improvePolicy(currentBelief);
	}

	/** Handle model changes */
	void handleChanges(
			std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
			bool resetTree) {

		bool hasDynamicChanges = options_.areDynamic;
		if (internalSimulation_) {
			simulator_->handleChanges(changes, hasDynamicChanges, resetTree);
		}
		if (hasDynamicChanges) {
			solver_->setChangeRoot(agent_->getCurrentBelief());
		} else {
			solver_->setChangeRoot(nullptr);
		}
		if (resetTree) {
			solver_->getModel()->applyChanges(changes, nullptr);
		} else {
			// The model only needs to inform the solver of changes if we intend
			// to keep the policy.
			solver_->getModel()->applyChanges(changes, solver_.get());
		}

		// Apply the changes, or simply reset the tree.
		if (resetTree) {
			solver_->resetTree(agent_->getCurrentBelief());
			agent_->setCurrentBelief(solver_->getPolicy()->getRoot());
		} else {
			solver_->applyChanges();
		}
	}

public:
	TapirNode(std::string rosNamespace = "") :
			internalSimulation_(false),
			agent_(nullptr),
			solverModel_(nullptr),
			stepNumber_(0),
			finished_(false),
			rosNamespace_(rosNamespace) {
	}

	virtual ~TapirNode() {
		if (!internalSimulation_) {
			delete agent_;
		}
	}

	virtual void initialiseBase(std::string problemName) {
		node_ = std::make_unique<ros::NodeHandle>(rosNamespace_);
		std::string problemPath = ros::package::getPath("tapir") +
						"/problems/" + problemName;
		loadOptions(problemPath);
		initTapir();
	}

};



#endif /* TAPIR_NODE_HPP_ */
