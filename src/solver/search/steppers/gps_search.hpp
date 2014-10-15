/** @file ucb_search.hpp
 *
 * Contains the necessary classes for a UCB-based search strategy; this is done via an
 * implementation of StepGenerator and StepGeneratorFactory; the latter can then be wrapped
 * inside a BasicSearchStrategy.
 */
#ifndef SOLVER_GPS_SEARCH_HPP_
#define SOLVER_GPS_SEARCH_HPP_

#include "solver/search/SearchStatus.hpp"
#include "solver/search/search_interface.hpp"

#include "solver/search/action-choosers/gps_choosers.hpp"

namespace solver {
/** A generator for steps that uses UCB to select actions.
 *
 * The action will be selected using UCB as long as the last action has been tried before; once
 * an action that has never been tried before is encountered, the search will terminate.
 */
class GpsStepGenerator : public StepGenerator {
public:
    /** Creates a new UcbStepGenerator associated with the given solver, and using the given
     * value for the UCB exploration coefficient.
     */
	GpsStepGenerator(SearchStatus &status, Solver *solver, choosers::GpsChooserOptions options);
    ~GpsStepGenerator() = default;
    _NO_COPY_OR_MOVE(GpsStepGenerator);

    virtual Model::StepResult getStep(HistoryEntry const *entry, State const *state, HistoricalData const *data) override;

private:
    /** The model to use to generate next steps. */
    Model *model;
    /** The exploration coefficient for UCB. */
    choosers::GpsChooserOptions options;

    /** True iff the last action selected hadn't been tried before. */
    bool choseUnvisitedAction;
};



/** A factory class for generating instances of UcbStepGenerator. */
class GpsStepGeneratorFactory: public StepGeneratorFactory {
public:
    /** Creates a new factory associated with the given solver, and with the given options. */
	GpsStepGeneratorFactory(Solver *solver, choosers::GpsChooserOptions options);
    virtual ~GpsStepGeneratorFactory() = default;
    _NO_COPY_OR_MOVE(GpsStepGeneratorFactory);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status, HistoryEntry const *entry, State const *state, HistoricalData const *data) override;
private:
    /** The associated solver. */
    Solver *solver;
    /** GPS search options */
    choosers::GpsChooserOptions options;
};

} /* namespace solver */

#endif /* SOLVER_GPS_SEARCH_HPP_ */
