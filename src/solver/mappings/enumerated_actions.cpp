#include "enumerated_actions.hpp"

#include <algorithm>
#include <cmath>

#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"
#include "solver/abstract-problem/Model.hpp"

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"

#include "ActionPool.hpp"
#include "ActionMapping.hpp"
#include "ObservationPool.hpp"

namespace solver {
/* ------------------- ModelWithEnumeratedActions ------------------- */
ModelWithEnumeratedActions::ModelWithEnumeratedActions(
        std::vector<std::unique_ptr<DiscretizedPoint>> allActions) :
        ModelWithDiscretizedActions(),
        allActions_(std::move(allActions)) {
}

long ModelWithEnumeratedActions::getNumberOfBins() {
    return allActions_.size();
}

std::unique_ptr<Action> ModelWithEnumeratedActions::sampleAnAction(
        long binNumber) {
    return allActions_[binNumber]->copy();
}

void ModelWithEnumeratedActions::setAllActions(
        std::vector<std::unique_ptr<DiscretizedPoint>> allActions) {
    allActions_ = std::move(allActions);
}
} /* namespace solver */



