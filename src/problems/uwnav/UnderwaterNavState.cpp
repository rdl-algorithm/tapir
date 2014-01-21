#include "UnderwaterNavState.hpp"

#include <cstddef>                      // for size_t


#include <functional>                   // for hash
#include <ostream>                      // for operator<<, ostream, basic_ostream
#include <vector>

#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/State.hpp"             // for State
#include "uwnav.hpp"

namespace uwnav {
template<class T>
void hash_combine(std::size_t &seed, T const &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

UnderwaterNavState::UnderwaterNavState(GridPosition pos) : position_(pos) {
}

UnderwaterNavState::UnderwaterNavState(long i, long j) : position_(i, j) {
}

UnderwaterNavState::UnderwaterNavState(UnderwaterNavState const &other) :
    solver::State(),
    position_(other.position_) {
}

UnderwaterNavState::UnderwaterNavState(UnderwaterNavState &&other) :
    solver::State(),
    position_(other.position_) {
}

double UnderwaterNavState::distanceTo(State const &otherState) const {
    UnderwaterNavState const *otherUnderwaterNavState =
        static_cast<UnderwaterNavState const *>(&otherState);
    return position_.manhattanDistanceTo(otherUnderwaterNavState->position_);
}

bool UnderwaterNavState::equals(State const &otherState) const {
    UnderwaterNavState const *otherUnderwaterNavState =
        static_cast<UnderwaterNavState const *>(&otherState);
    return position_ == otherUnderwaterNavState->position_;
}

std::size_t UnderwaterNavState::hash() const {
    std::size_t hashValue = 0;
    hash_combine(hashValue, position_.i);
    hash_combine(hashValue, position_.j);
    return hashValue;
}

virtual std::vector<double> UnderwaterNavState::asVector() const {
    std::vector<double> vec(2);
    vec[0] = position_.i;
    vec[1] = position_.j;
    return vec;
}

void UnderwaterNavState::print(std::ostream &os) const {
    os << position_;
}
} /* namespace uwnav */
