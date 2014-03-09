#include "VectorLPTextSerializer.hpp"

#include <vector>

#include "solver/abstract-problem/Vector.hpp"
#include "solver/abstract-problem/VectorLP.hpp"

namespace solver {
void VectorLPTextSerializer::save(std::vector<double> const &vector, std::ostream &os) {
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

void VectorLPTextSerializer::load(std::vector<double> &vector, std::istream &is) {
    std::string tmpStr;
    std::getline(is, tmpStr, '<');
    std::getline(is, tmpStr, '>');

    vector.clear();

    if (!is.good() || tmpStr.empty()) {
        return;
    }

    std::istringstream sstr(tmpStr);
    while (sstr.good()) {
        double tmpDouble;
        sstr >> tmpDouble;
        vector.push_back(tmpDouble);
    }
}

void VectorLPTextSerializer::saveState(State const *state, std::ostream &os) {
    VectorLP const &vector = static_cast<VectorLP const &>(*state);
    os << vector.p_;
    save(vector.asVector(), os);
}

std::unique_ptr<State> VectorLPTextSerializer::loadState(std::istream &is) {
    std::vector<double> vector;
    double p;
    is >> p;
    load(vector, is);
    if (vector.empty()) {
        return nullptr;
    }
    return std::make_unique<VectorLP>(vector, p);
}

void VectorLPTextSerializer::saveObservation(Observation const *obs, std::ostream &os) {
    if (obs == nullptr) {
        os << " 1 <>";
        return;
    }
    VectorLP const &vector = static_cast<VectorLP const &>(*obs);
    os << vector.p_;
    save(vector.asVector(), os);
}

std::unique_ptr<Observation> VectorLPTextSerializer::loadObservation(std::istream &is) {
    std::vector<double> vector;
    double p;
    is >> p;
    load(vector, is);
    if (vector.empty()) {
        return nullptr;
    }
    return std::make_unique<VectorLP>(vector, p);
}

void VectorLPTextSerializer::saveAction(Action const *action, std::ostream &os) {
    if (action == nullptr) {
        os << " 1 <>";
        return;
    }
    VectorLP const &vector = static_cast<VectorLP const &>(*action);
    os << vector.p_;
    save(vector.asVector(), os);
}

std::unique_ptr<Action> VectorLPTextSerializer::loadAction(std::istream &is) {
    std::vector<double> vector;
    double p;
    is >> p;
    load(vector, is);
    if (vector.empty()) {
        return nullptr;
    }
    return std::make_unique<VectorLP>(vector, p);
}
} /* namespace solver */
