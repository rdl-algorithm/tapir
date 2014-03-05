#pragma once

#include "solver/HistoryCorrector.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/geometry/State.hpp"
#include "solver/ChangeFlags.hpp"
#include "solver/StatePool.hpp"
#include "Nav2DModel.hpp"


namespace nav2d {

namespace changes{
using namespace solver::changes;
}

class Nav2DSpcHistoryCorrector: public solver::HistoryCorrector {
public:
	typedef solver::HistorySequence HistorySequence;
	typedef nav2d::Nav2DModel Model;
	typedef solver::HistoryEntry HistoryEntry;
	typedef solver::State State;
	typedef solver::ChangeFlags ChangeFlags;
	typedef solver::StateInfo StateInfo;
	typedef solver::Observation Observation;
public:
	Nav2DSpcHistoryCorrector(Model *model);
    virtual ~Nav2DSpcHistoryCorrector() = default;

    void reviseSequence(HistorySequence *sequence);
};


} // namespace nav2d
