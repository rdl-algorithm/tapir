#include "VectorState.hpp"

#include <cmath>
#include <cstddef>

#include <vector>

namespace solver {
template<class T>
void hash_combine(std::size_t &seed, T const &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

double VectorState::distanceTo(State const &otherState) const {
    std::vector<double> v1 = this->asVector();
    std::vector<double> v2 = static_cast<VectorState const *>(
            &otherState)->asVector();
    std::vector<double>::iterator i1 = v1.begin();
    std::vector<double>::iterator i2 = v2.begin();

    double distance = 0;
    for (; i1 < v1.end(); i1++, i2++) {
        distance += std::abs(*i1 - *i2);
    }
    return distance;
}

bool VectorState::equals(State const &otherState) const {
    std::vector<double> v1 = this->asVector();
    std::vector<double> v2 = static_cast<VectorState const *>(
            &otherState)->asVector();
    std::vector<double>::iterator i1 = v1.begin();
    std::vector<double>::iterator i2 = v2.begin();

    for (; i1 < v1.end(); i1++, i2++) {
        if (*i1 == *i2) {
            return false;
        }
    }
    return true;
}

std::size_t VectorState::hash() const {
    std::vector<double> v1 = this->asVector();

    std::size_t hashValue = 0;
    for (std::vector<double>::iterator i1 = v1.begin(); i1 < v1.end(); i1++) {
        hash_combine(hashValue, *i1);
    }
    return hashValue;

}
} /* namespace solver */
