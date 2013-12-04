#ifndef ROCKSAMPLESTATE_HPP
#define ROCKSAMPLESTATE_HPP

#include <cstddef>                      // for size_t

#include "State.hpp"                    // for State

template<class T>
void hash_combine(std::size_t &seed, const T &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

class RockSampleState: public State {
public:
    RockSampleState(GridPosition position, std::vector<bool> isRockGood) :
                position(position),
                isRockGood(isRockGood) {
    }
    virtual ~RockSampleState() = default;
    RockSampleState(const RockSampleState&) = delete;
    RockSampleState(RockSampleState&&) = delete;
    virtual RockSampleState &operator=(const RockSampleState&) = delete;
    virtual RockSampleState &operator=(RockSampleState&&) = delete;

    GridPosition getPosition() {
        return position;
    }

    std::vector<bool> isRockGood() {
        return isRockGood;
    }

    virtual double distanceTo(const State &otherState) {
        const RockSampleState *otherRockState =
                dynamic_cast<const RockSampleState*>(&otherState);
        double distance = position.distanceTo(otherRockState->position);
        distance /= 10.0;
        typedef std::vector<bool>::const_iterator BoolIt;
        BoolIt it1 = isRockGood.cbegin();
        BoolIt it2 = otherRockState->isRockGood.cbegin();
        for (; it1 != isRockGood.cend(); it1++, it2++) {
            if (*it1 != *it2) {
                distance += 1;
            }
        }
        return distance;
    }

    virtual bool equals(const State &otherState) {
        const RockSampleState *otherRockState =
                        dynamic_cast<const RockSampleState*>(&otherState);
        return (position == otherRockState->position &&
                isRockGood == otherRockState->isRockGood);
    }

    virtual std::size_t hash() const {
        std::size_t hashValue = 0;
        hash_combine(hashValue, position.i);
        hash_combine(hashValue, position.j);
        hash_combine(hashValue, isRockGood);
        return hashValue;
    }

    virtual void print(std::ostream &os) {
        os << position << " GOOD: {";
        std::vector<int> goodRocks;
        std::vector<int> badRocks;
        for (std::size_t i = 0; i < isRockGood.size(); i++) {
            if (isRockGood[i]) {
                goodRocks.push_back(i);
            } else {
                badRocks.push_back(i);
            }
        }
        std::copy(goodRocks.begin(), goodRocks.end(),
                std::ostream_iterator<double>(os, " "));
        os << "}; BAD: {";
        std::copy(badRocks.begin(), badRocks.end(),
                std::ostream_iterator<double>(os, " "));
        os << "}";
    }

private:
    GridPosition position;
    std::vector<bool> isRockGood;
};
#endif /* ROCKSAMPLESTATE_HPP */
