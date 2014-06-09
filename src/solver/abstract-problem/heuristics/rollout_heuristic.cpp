#include "solver/abstract-problem/heuristics/rollout_heuristic.hpp"

#include <memory>

#include "solver/HistoryEntry.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/search/SearchStatus.hpp"

namespace solver {
RolloutHeuristic::RolloutHeuristic(Model *model, std::unique_ptr<StepGeneratorFactory> factory,
        Heuristic heuristic) :
        model_(model),
        factory_(std::move(factory)),
        heuristic_(heuristic) {
}
double RolloutHeuristic::getHeuristicValue(HistoryEntry const *entry,
        State const *state, HistoricalData const *data) {
    SearchStatus status = SearchStatus::UNINITIALIZED;
    std::unique_ptr<StepGenerator> generator = factory_->createGenerator(status,
            entry, state, data);
    double value = 0.0;
    double netDiscount = 1.0;
    double discountFactor = model_->getDiscountFactor();

    Model::StepResult result = generator->getStep(entry, state, data);
    std::unique_ptr<State> currentState = state->copy();
    std::unique_ptr<HistoricalData> currentData = nullptr;
    while (result.action != nullptr) {
        value += netDiscount * result.reward;
        netDiscount *= discountFactor;

        currentState = result.nextState->copy();
        if (data != nullptr) {
            currentData = data->createChild(*result.action, *result.observation);
            data = currentData.get();
        }
        result = generator->getStep(nullptr, result.nextState.get(), data);
    }
    value += netDiscount * heuristic_(nullptr, currentState.get(), data);
    return value;
}
} /* namespace solver */
