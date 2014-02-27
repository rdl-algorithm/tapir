#include "VectorTextSerializer.hpp"

#include <vector>

#include "solver/geometry/Vector.hpp"
#include "solver/geometry/VectorLP.hpp"

namespace solver {
void VectorTextSerializer::save(std::vector<double> const &vector, std::ostream &os) {
    os << "<";
    for (std::vector<double>::const_iterator it = vector.begin();
            it != vector.end(); it++) {
        os << *it;
        if (it + 1 != vector.end()) {
            os << " ";
        }
    }
    os << ">";
}

void VectorTextSerializer::load(std::vector<double> &vector, std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '<');
    std::getline(is, tmpStr, '>');

    vector.clear();

    if (!is.good() || tmpStr.empty()) {
        return;
    }

    std::stringstream sstr(tmpStr);
    while (sstr.good()) {
        double tmpDouble;
        sstr >> tmpDouble;
        vector.push_back(tmpDouble);
    }
}

void VectorTextSerializer::saveState(State const *state, std::ostream &os) {
    std::vector<double> vector = static_cast<Vector const *>(state)->asVector();
    save(vector, os);
}

std::unique_ptr<State> VectorTextSerializer::loadState(std::istream &is) {
    std::vector<double> vector;
    load(vector, is);
    if (vector.empty()) {
        return nullptr;
    }
    return std::make_unique<VectorLP>(vector, 1);
}

void VectorTextSerializer::saveObservation(Observation const *obs, std::ostream &os) {
    if (obs == nullptr) {
        os << "<>";
    } else {
        std::vector<double> vector = static_cast<Vector const *>(obs)->asVector();
        save(vector, os);
    }
}

std::unique_ptr<Observation> VectorTextSerializer::loadObservation(std::istream &is) {
    std::vector<double> vector;
    load(vector, is);
    if (vector.empty()) {
        return nullptr;
    }
    return std::make_unique<VectorLP>(vector, 1);
}

void VectorTextSerializer::saveAction(Action const *action, std::ostream &os) {
    if (action == nullptr) {
        os << "<>";
    } else {
        std::vector<double> vector = static_cast<Vector const *>(action)->asVector();
        save(vector, os);
    }
}

std::unique_ptr<Action> VectorTextSerializer::loadAction(std::istream &is) {
    std::vector<double> vector;
    load(vector, is);
    if (vector.empty()) {
        return nullptr;
    }
    return std::make_unique<VectorLP>(vector, 1);
}
} /* namespace solver */
