[TOC]

[Tag]: @ref src/problems/tag
[RockSample]: @ref src/problems/rocksample


[Solver]: @ref solver::Solver


[Action]: @ref solver::Action
[Model]: @ref solver::Model
[ModelChange]: @ref solver::ModelChange
[Heuristic]: @ref solver::Heuristic
[Observation]: @ref solver::Observation
[Options]: @ref solver::Options
[State]: @ref solver::State
[TransitionParameters]: @ref solver::TransitionParameters
[Vector]: @ref solver::Vector
[VectorState]: @ref solver::VectorState


[Model::isTerminal]: @ref solver::Model::isTerminal
[Model::sampleAnInitState]: @ref solver::Model::sampleAnInitState
[Model::sampleStateUninformed]: @ref solver::Model::sampleStateUninformed
[Model::applyChanges]: @ref solver::Model::applyChanges
[Model::getHeuristicFunction]: @ref solver::Model::getHeuristicFunction

[Model::generateParticles]: @ref solver::Model::generateParticles
[Model::generateParticles2]: @ref solver::Model::generateParticles(BeliefNode*, Action const &, Observation const &, long)

[Model::generateStep]: @ref solver::Model::generateStep
[Model::generateTransition]: @ref solver::Model::generateTransition
[Model::generateNextState]: @ref solver::Model::generateNextState
[Model::generateObservation]: @ref solver::Model::generateObservation
[Model::generateReward]: @ref solver::Model::generateReward

[Model::createActionPool]: @ref solver::Model::createActionPool
[Model::createStateIndex]: @ref solver::Model::createStateIndex


[Options::discountFactor]: @ref solver::Options::discountFactor
[Options::numberOfStateVariables]: @ref solver::Options::numberOfStateVariables


[copy()]: @ref solver::Point::copy
[equals()]: @ref solver::Point::equals
[hash()]: @ref solver::Point::hash
[distanceTo()]: @ref solver::Point::distanceTo
[print()]: @ref solver::Point::print


[StatePool]: @ref solver::StatePool
[StateIndex]: @ref solver::StateIndex
[RTree]: @ref solver::RTree
[Vector::asVector]: @ref solver::Vector::asVector


[ActionPool]: @ref solver::ActionPool
[ActionMapping]: @ref solver::ActionMapping
[ActionMapping::getNextActionToTry]: @ref solver::ActionMapping::getNextActionToTry
[UCB search strategy]: @ref solver::UcbStepGenerator::getStep

[EnumeratedActionPool]: @ref solver::EnumeratedActionPool
[DiscretizedPoint]: @ref solver::DiscretizedPoint
[DiscretizedPoint::getBinNumber]: @ref solver::DiscretizedPoint::getBinNumber


[HistoryCorrector]: @ref solver::HistoryCorrector
[DefaultHistoryCorrector]: @ref solver::DefaultHistoryCorrector



[Serializer]: @ref solver::Serializer
[TextSerializer]: @ref solver::TextSerializer
[DiscreteObservationPool]: @ref solver::DiscreteObservationPool
[DiscreteObservationTextSerializer]: @ref solver::DiscreteObservationTextSerializer
[EnumeratedActionPool]: @ref solver::EnumeratedActionPool
[EnumeratedActionTextSerializer]: @ref solver::EnumeratedActionTextSerializer
[TagTextSerializer]: @ref tag::TagTextSerializer



[ModelWithProgramOptions]: @ref shared::ModelWithProgramOptions
[SharedOptions]: @ref shared::SharedOptions

[Tag_config]: ../src/problems/tag/tests/default.cfg
[TagOptions]: @ref tag::TagOptions
[TagModel]: @ref tag::TagModel
[TagState]: @ref tag::TagState
[TagAction]: @ref tag::TagAction
[TagObservation]: @ref tag::TagObservation

[TagChange]: @ref tag::TagChange
[TagTextSerializer::saveModelChange]: @ref tag::TagTextSerializer::saveModelChange
[TagTextSerializer::loadModelChange]: @ref tag::TagTextSerializer::loadModelChange
[TagModel::applyChanges]: @ref tag::TagModel::applyChanges



[solve.hpp]: @ref solve.hpp
[simulate.hpp]: @ref simulate.hpp
[Tag_solve.cpp]: @ref src/problems/tag/solve.cpp
[Tag_simulate.cpp]: @ref src/problems/tag/simulate.cpp


[Makefile]: ../Makefile
[Makefile_src]: ../src/Makefile
[Makefile_solver]: ../src/solver/Makefile
[Build README]: @ref build-system
[build.mk]: ../.make/build.mk
[stack.mk]: ../.make/stack.mk
[template.mk]: ../.make/template.mk
[problem-template.mk]: ../.make/problem-template.mk
