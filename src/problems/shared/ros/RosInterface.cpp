#include "RosInterface.hpp"

using namespace solver;
using std::cout;
using std::endl;

RosInterface::RosInterface() :
	initialised_(false),
	internalSimulation_(false), // set in initialise
	hasDynamicChanges_(false),	 // set in initialise
	agent_(nullptr) {

}

RosInterface::~RosInterface() {
	if (!internalSimulation_) {
		delete agent_;
	}
}

bool RosInterface::stepSimulation() {
	if (!initialised_) {
		std::cerr << "Error: Initialise first." << endl;
		return false;
	} else if (!internalSimulation_) {
		std::cerr << "Error: Not allowed unless simulating internally." << endl;
		return false;
	}
	return simulator_->stepSimulation();
}

bool RosInterface::handleChanges(
		std::vector<std::unique_ptr<solver::ModelChange>> const &changes,
		bool resetTree) {
	if (internalSimulation_) {
		return simulator_->handleChanges(changes, hasDynamicChanges_, resetTree);
	}

	if (hasDynamicChanges_) {
		solver_->setChangeRoot(agent_->getCurrentBelief());
	} else {
		solver_->setChangeRoot(nullptr);
	}

	if (resetTree) {
		solver_->getModel()->applyChanges(changes, nullptr);
	} else {
		// The model only needs to inform the solver of changes if we intend to keep the policy.
		solver_->getModel()->applyChanges(changes, solver_.get());
	}

	// Apply the changes, or simply reset the tree.
	if (resetTree) {
		solver_->resetTree(agent_->getCurrentBelief());
		agent_->setCurrentBelief(solver_->getPolicy()->getRoot());
	} else {
		solver_->applyChanges();
	}
	return true;
}

solver::Model *RosInterface::getSimulatorModel() const {
	return simulator_->getModel();
}

solver::Model *RosInterface::getSolverModel() const {
	return solver_->getModel();
}

solver::Agent *RosInterface::getAgent() const {
	return agent_;
}

solver::Solver *RosInterface::getSolver() const {
    return solver_.get();
}

solver::Simulator *RosInterface::getSimulator() const {
    return simulator_.get();
}

